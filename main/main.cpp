/* Includes ---------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <math.h> // Required for fabsf
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// Wrap C driver headers in extern "C" so C++ can read them
extern "C" {
    #include "config.h"
    #include "w5500_driver.h"
    #include "mic_driver.h"
}

// Edge Impulse Include
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

static const char *TAG = "MAIN";

// --- CONFIGURATION ---
#define CONFIDENCE_THRESHOLD 0.75f
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// --- APP STATES ---
typedef enum {
    APP_STATE_INIT,
    APP_STATE_IDLE,
    APP_STATE_RUNNING
} app_state_t;

static volatile app_state_t current_state = APP_STATE_INIT;

// --- EDGE IMPULSE CALLBACK ---
static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, inference_buffer + offset, length * sizeof(float));
    return 0;
}

// --- AUDIO + AI TASK ---
void audio_ai_task(void *arg) {
    // Buffers for Driver (Stereo)
    static int32_t raw_buffer[BUFFER_SAMPLES * 2]; 
    
    // Pointer to where we are currently writing in the AI buffer
    size_t inference_ptr = 0;

    while (1) {
        if (current_state == APP_STATE_RUNNING) {
            
            // 1. READ MICROPHONE (Blocking I2S Read)
            // Returns TOTAL samples (Left + Right interleaved)
            size_t samples_read = mic_read_stereo(raw_buffer, BUFFER_SAMPLES * 2);

            float energy_sum = 0.0f; // <--- Fixed: Variable declared here

            // 2. PROCESS & BUFFER
            // Loop through PAIRS (Stereo). i goes up to samples_read / 2
            for (int i = 0; i < samples_read / 2; i++) {
                
                // Extract LEFT channel (Index 0, 2, 4...)
                // FIX: Valid indices are now ensured
                int32_t sample = raw_buffer[i * 2] >> AUDIO_GAIN;

                // Calculate Energy for Debugging
                energy_sum += fabsf((float)sample);

                // Hard Limit (Clip)
                if (sample > 32767) sample = 32767;
                if (sample < -32768) sample = -32768;

                // Add to AI Buffer (Convert to float)
                if (inference_ptr < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                    inference_buffer[inference_ptr++] = (float)sample;
                }
            }

            // --- DEBUG PRINT: Check Volume ---
            // Only print every 20th chunk to avoid spamming
            static int debug_skip = 0;
            if (debug_skip++ > 20) {
                float avg_vol = energy_sum / (samples_read / 2);
                ESP_LOGI(TAG, "Mic Volume: %.2f", avg_vol);
                debug_skip = 0;
            }

            // 3. RUN INFERENCE (When buffer is full ~1 second)
            if (inference_ptr >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                
                signal_t signal;
                signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
                signal.get_data = &raw_feature_get_data;

                ei_impulse_result_t result = { 0 };
                EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);

                if (r != EI_IMPULSE_OK) {
                    ESP_LOGE(TAG, "Classifier failed (%d)", r);
                } else {
                    // Print Probabilities
                    ei_printf("DSP: %d ms, Inf: %d ms | ", result.timing.dsp, result.timing.classification);
                    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                        ei_printf("%s: %.2f  ", result.classification[ix].label, result.classification[ix].value);
                    }
                    ei_printf("\n");

                    // Check for Gunshot
                    float gunshot_score = 0.0f;
                    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                        if (strcmp(result.classification[ix].label, "gunshot") == 0) {
                            gunshot_score = result.classification[ix].value;
                        }
                    }

                    if (gunshot_score > CONFIDENCE_THRESHOLD) {
                        ESP_LOGW(TAG, "!!! GUNSHOT DETECTED (Confidence: %.2f) !!!", gunshot_score);
                        // Trigger Ethernet alert here if needed
                    }
                }
                inference_ptr = 0; // Reset buffer
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// --- UI TASK ---
void ui_task(void *arg) {
    bool last_btn = true;
    while(1) {
        bool btn = gpio_get_level(PIN_BUTTON);
        if (last_btn && !btn) { // Falling Edge
            if (current_state == APP_STATE_IDLE) {
                ESP_LOGI(TAG, "Starting Detector...");
                gpio_set_level(PIN_LED, 1);
                current_state = APP_STATE_RUNNING;
            } else {
                ESP_LOGI(TAG, "Stopping Detector...");
                gpio_set_level(PIN_LED, 0);
                current_state = APP_STATE_IDLE;
            }
            vTaskDelay(pdMS_TO_TICKS(250)); // Debounce
        }
        last_btn = btn;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- MAIN ---
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "System Init - Gunshot Detector");

    gpio_reset_pin(PIN_LED); 
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(PIN_BUTTON); 
    gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON, GPIO_PULLUP_ONLY); // <--- FIX: Prevent random stops

    // eth_init(); // Uncomment when ready to use Ethernet
    mic_init();

    // Stack size 8192 is required for Neural Net
    xTaskCreatePinnedToCore(audio_ai_task, "AI_Task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(ui_task, "UI_Task", 2048, NULL, 1, NULL, 0);

    current_state = APP_STATE_IDLE;
    ESP_LOGI(TAG, "System Ready.");
}