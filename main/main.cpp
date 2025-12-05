/* Includes ---------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

// REMOVED: lwip headers (We don't need sockets anymore!)

extern "C" {
    #include "config.h"
    #include "w5500_driver.h"
    #include "mic_driver.h"
}

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

static const char *TAG = "MAIN";

// --- CONFIGURATION ---
#define CONFIDENCE_THRESHOLD 0.75f
#define QUEUE_LENGTH 10 
#define AI_BUFFER_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE

// --- MAGIC PACKET SIGNATURE ---
// We will send these specific values to signal a gunshot
#define MAGIC_KEY_1  0x7F01 
#define MAGIC_KEY_2  0x7F02

// --- DATA STRUCTURES ---
typedef struct {
    int16_t samples[BUFFER_SAMPLES * 2];
    size_t count;
} audio_chunk_t;

static QueueHandle_t audio_net_queue = NULL;
static QueueHandle_t audio_ai_queue = NULL;

typedef enum { APP_STATE_IDLE, APP_STATE_RUNNING } app_state_t;
static volatile app_state_t current_state = APP_STATE_IDLE;

// --- TASK 1: MICROPHONE (Producer) ---
void mic_task(void *arg) {
    static int32_t raw_buffer[BUFFER_SAMPLES * 2]; 
    
    while (1) {
        if (current_state == APP_STATE_RUNNING) {
            size_t samples_read = mic_read_stereo(raw_buffer, BUFFER_SAMPLES * 2);

            audio_chunk_t chunk;
            chunk.count = 0;

            for (int i = 0; i < samples_read / 2; i++) {
                int32_t left = raw_buffer[i * 2] >> AUDIO_GAIN; 
                int32_t right = raw_buffer[i * 2 + 1] >> AUDIO_GAIN;

                // Clamp
                if (left > 32767) left = 32767; else if (left < -32768) left = -32768;
                if (right > 32767) right = 32767; else if (right < -32768) right = -32768;

                chunk.samples[chunk.count++] = (int16_t)left;
                chunk.samples[chunk.count++] = (int16_t)right;
            }

            if (audio_net_queue) xQueueSend(audio_net_queue, &chunk, 0);
            if (audio_ai_queue)  xQueueSend(audio_ai_queue, &chunk, 0);
            
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// --- TASK 2: NETWORK (UDP Stream) ---
// This handles BOTH Audio and Alerts now (Safe & Fast)
void net_task(void *arg) {
    audio_chunk_t chunk;
    while (1) {
        if (xQueueReceive(audio_net_queue, &chunk, portMAX_DELAY) == pdTRUE) {
            eth_send_rtp_packet(chunk.samples, chunk.count);
        }
    }
}

// --- TASK 3: AI INFERENCE ---
static float rb_buffer[AI_BUFFER_SIZE]; 
static size_t rb_head = 0;           

static int ring_buffer_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t rb_len = AI_BUFFER_SIZE;
    size_t start_idx = (rb_head + offset) % rb_len; 
    
    if (start_idx + length < rb_len) {
        memcpy(out_ptr, &rb_buffer[start_idx], length * sizeof(float));
    } else {
        size_t part1 = rb_len - start_idx;
        memcpy(out_ptr, &rb_buffer[start_idx], part1 * sizeof(float));
        memcpy(out_ptr + part1, &rb_buffer[0], (length - part1) * sizeof(float));
    }
    return 0;
} 

#define AI_SOFTWARE_GAIN 1.0f 

//
// Ensure you have #include "esp_timer.h" at the top of your file

void ai_task(void *arg) {
    audio_chunk_t chunk;
    bool trigger_active = false;
    int post_trigger_samples = 0;
    int trigger_cooldown = 0;
    const int COOLDOWN_SAMPLES = 24000; 

    ESP_LOGI(TAG, "AI Task Started");

    // 1. Static variable to remember start time across loops
    static int64_t t_trigger = 0;

    while (1) {
        if (xQueueReceive(audio_ai_queue, &chunk, portMAX_DELAY) == pdTRUE) {
            
            for (int i = 0; i < chunk.count; i += 2) {
                float sample = (float)chunk.samples[i] * AI_SOFTWARE_GAIN;
                rb_buffer[rb_head] = sample;
                rb_head++;
                if (rb_head >= AI_BUFFER_SIZE) rb_head = 0; 
                
                if (!trigger_active && trigger_cooldown <= 0) {
                    if (fabs(sample) > 25000) { 
                        // Start the stopwatch when loud noise occurs
                        t_trigger = esp_timer_get_time();
                        ESP_LOGW(TAG, ">>> TRIGGER! Level: %.0f <<<", sample);
                        trigger_active = true;
                        post_trigger_samples = 16000 * 0.5; 
                    }
                }
            }

            if (trigger_cooldown > 0) trigger_cooldown -= (chunk.count / 2);

            if (trigger_active) {
                post_trigger_samples -= (chunk.count / 2);

                if (post_trigger_samples <= 0) {
                    
                    signal_t signal;
                    signal.total_length = AI_BUFFER_SIZE;
                    signal.get_data = &ring_buffer_get_data; 
                    ei_impulse_result_t result = { 0 };
                    
                    // --- INFERENCE START ---
                    if (run_classifier(&signal, &result, false) == EI_IMPULSE_OK) {
                        
                        // 2. STOPWATCH CHECKPOINT A: Processing Complete
                        // This measures purely how long the AI took to think.
                        int64_t t_processed = esp_timer_get_time();
                        int64_t processing_time = t_processed - t_trigger;

                        float score = 0.0f;
                        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                            if (strcmp(result.classification[ix].label, "gunshot") == 0) {
                                score = result.classification[ix].value;
                            }
                        }
                        ESP_LOGI(TAG, "AI RESULT: %.2f", score);
                        
                        if (score > CONFIDENCE_THRESHOLD) {
                             ESP_LOGE(TAG, "!!! CONFIRMED GUNSHOT !!!");
                             
                             // Send 'Magic Packet'
                             audio_chunk_t alert;
                             alert.count = 4;
                             alert.samples[0] = MAGIC_KEY_1; 
                             alert.samples[1] = MAGIC_KEY_2;
                             alert.samples[2] = MAGIC_KEY_1; 
                             alert.samples[3] = MAGIC_KEY_2;
                             
                             // Send multiple times
                             for(int k=0; k<5; k++) {
                                if(audio_net_queue) xQueueSend(audio_net_queue, &alert, 0);
                                vTaskDelay(pdMS_TO_TICKS(5));
                             }

                             // 3. STOPWATCH CHECKPOINT B: Network Complete
                             // This measures the total time including the network queue delay.
                             int64_t t_sent = esp_timer_get_time();
                             int64_t total_latency = t_sent - t_trigger;

                             // LOG BOTH METRICS
                             // This answers: "How fast is the AI?" AND "How fast is the whole system?"
                             ESP_LOGI(TAG, "[TEST_DATA] TYPE:GUNSHOT | PROC_TIME:%lld us | TOTAL_LATENCY:%lld us", processing_time, total_latency);
                        }
                        else {
                            // Noise/Clap rejected
                            // We only have processing time here (since we didn't send anything)
                            ESP_LOGW(TAG, "[TEST_DATA] TYPE:NOISE   | PROC_TIME:%lld us | TOTAL_LATENCY: N/A", processing_time); 
                        }
                    }
                    // --- INFERENCE END ---

                    trigger_active = false;
                    trigger_cooldown = COOLDOWN_SAMPLES; 
                }
            }
        }
    }
}

// --- UI TASK ---
void ui_task(void *arg) {
    bool last_btn = true;
    while(1) {
        bool btn = gpio_get_level(PIN_BUTTON);
        if (last_btn && !btn) { 
            if (current_state == APP_STATE_IDLE) {
                ESP_LOGI(TAG, "Starting...");
                xQueueReset(audio_net_queue);
                xQueueReset(audio_ai_queue);
                gpio_set_level(PIN_LED, 1);
                current_state = APP_STATE_RUNNING;
            } else {
                ESP_LOGI(TAG, "Stopping...");
                gpio_set_level(PIN_LED, 0);
                current_state = APP_STATE_IDLE;
            }
            vTaskDelay(pdMS_TO_TICKS(250)); 
        }
        last_btn = btn;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- MAIN ---
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "System Init");
    
    gpio_reset_pin(PIN_LED); gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_BUTTON); gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON, GPIO_PULLUP_ONLY);

    audio_net_queue = xQueueCreate(QUEUE_LENGTH, sizeof(audio_chunk_t));
    audio_ai_queue  = xQueueCreate(QUEUE_LENGTH, sizeof(audio_chunk_t));

    eth_init();
    mic_init();

    xTaskCreatePinnedToCore(mic_task, "MicTask", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(net_task, "NetTask", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ai_task, "AiTask", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(ui_task, "UiTask", 2048, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "System Ready.");
}