#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "w5500_driver.h"
#include "mic_driver.h"

static const char *TAG = "MAIN";

// --- APP STATES ---
typedef enum {
    APP_STATE_INIT,
    APP_STATE_IDLE,
    APP_STATE_STREAMING,
    APP_STATE_ERROR
} app_state_t;

static volatile app_state_t current_state = APP_STATE_INIT;

// --- DATA TASK (The Heavy Lifter) ---
void audio_process_task(void *arg) {
    static int32_t raw_buffer[BUFFER_SAMPLES * 2]; // Stereo Raw
    static int16_t pcm_buffer[BUFFER_SAMPLES];     // Processed

    while (1) {
        if (current_state == APP_STATE_STREAMING) {
            
            // 1. Acquire Data (Blocking)
            size_t samples = mic_read_stereo(raw_buffer, BUFFER_SAMPLES * 2);

            // 2. Process Data (Gain + Limiter)
            for (int i = 0; i < samples; i++) {
                int32_t val = raw_buffer[i] >> AUDIO_GAIN;
                if (val > 32767) val = 32767;
                if (val < -32768) val = -32768;
                pcm_buffer[i] = (int16_t)val;
            }

            // 3. Transmit Data
            eth_send_rtp_packet(pcm_buffer, samples);
            
        } else {
            // Save power when not streaming
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }
    }
}

// --- INPUT TASK (User Interface) ---
void ui_task(void *arg) {
    bool last_btn = true;
    
    while(1) {
        bool btn = gpio_get_level(PIN_BUTTON);
        
        if (last_btn && !btn) { // Falling Edge (Press)
            // State Transition Logic
            switch (current_state) {
                case APP_STATE_IDLE:
                    ESP_LOGI(TAG, "UI: Starting Stream...");
                    gpio_set_level(PIN_LED, 1);
                    current_state = APP_STATE_STREAMING;
                    break;

                case APP_STATE_STREAMING:
                    ESP_LOGI(TAG, "UI: Stopping Stream...");
                    gpio_set_level(PIN_LED, 0);
                    current_state = APP_STATE_IDLE;
                    break;
                    
                default:
                    break;
            }
            vTaskDelay(pdMS_TO_TICKS(250)); // Debounce
        }
        last_btn = btn;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    // --- STATE: INIT ---
    ESP_LOGI(TAG, "System Initializing...");
    
    // 1. Setup IO
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_BUTTON);
    gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
    
    // 2. Setup Drivers
    eth_init();
    mic_init();

    // 3. Create Tasks
    // Audio task on Core 1 (High Perf), UI on Core 0
    xTaskCreatePinnedToCore(audio_process_task, "AudioTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(ui_task, "UITask", 2048, NULL, 1, NULL, 0);

    // Transition to Idle
    current_state = APP_STATE_IDLE;
    ESP_LOGI(TAG, "System Ready (IDLE)");
}