/* Includes ---------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

// Wrap C driver headers in extern "C"
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
#define QUEUE_LENGTH 10  // How many audio chunks can be buffered
#define AI_BUFFER_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE

// --- DATA STRUCTURES ---
// A single chunk of audio to be passed between tasks
typedef struct {
    int16_t samples[BUFFER_SAMPLES]; // Mono 16-bit samples
    size_t count;
} audio_chunk_t;

// Queues
static QueueHandle_t audio_net_queue = NULL;
static QueueHandle_t audio_ai_queue = NULL;

// --- APP STATES ---
typedef enum { APP_STATE_IDLE, APP_STATE_RUNNING } app_state_t;
static volatile app_state_t current_state = APP_STATE_IDLE;

// --- TASK 1: MICROPHONE (Producer) ---
// Runs at HIGH Priority. Does nothing but read I2S and push to queues.
void mic_task(void *arg) {
    static int32_t raw_buffer[BUFFER_SAMPLES * 2]; // Stereo 32-bit Raw
    
    while (1) {
        if (current_state == APP_STATE_RUNNING) {
            // 1. Blocking Read (Hardware timed)
            size_t samples_read = mic_read_stereo(raw_buffer, BUFFER_SAMPLES * 2);

            // 2. Prepare Chunk (Convert to Mono 16-bit immediately)
            audio_chunk_t chunk;
            chunk.count = 0;

            for (int i = 0; i < samples_read / 2; i++) {
                int32_t s32 = raw_buffer[i * 2] >> AUDIO_GAIN;
                if (s32 > 32767) s32 = 32767;
                if (s32 < -32768) s32 = -32768;
                chunk.samples[chunk.count++] = (int16_t)s32;
            }

            // 3. Push to Queues (Non-blocking if full)
            // We use xQueueSend to copy the chunk into the queue.
            if (audio_net_queue) xQueueSend(audio_net_queue, &chunk, 0);
            if (audio_ai_queue)  xQueueSend(audio_ai_queue, &chunk, 0);
            
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// --- TASK 2: NETWORK (Consumer A) ---
// Sends UDP packets. If it gets stuck, it won't block the mic.
void net_task(void *arg) {
    audio_chunk_t chunk;
    while (1) {
        if (xQueueReceive(audio_net_queue, &chunk, portMAX_DELAY) == pdTRUE) {
            eth_send_rtp_packet(chunk.samples, chunk.count);
        }
    }
}

// --- TASK 3: AI INFERENCE (Consumer B) ---
// Accumulates 1 second of audio, then runs heavy inference.
static float inference_buffer[AI_BUFFER_SIZE];

static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, inference_buffer + offset, length * sizeof(float));
    return 0;
}

// --- RING BUFFER VARIABLES ---
#define AI_BUFFER_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE // 16000 samples (1 sec)
static float rb_buffer[AI_BUFFER_SIZE]; // Circular Buffer
static size_t rb_head = 0;              // Current write position

// This function "unwraps" the ring buffer for the AI
static int ring_buffer_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t rb_len = AI_BUFFER_SIZE;
    // We want to read starting from: (Head - Window_Size + Offset)
    // Effectively looking "backwards" from the current head
    size_t start_idx = (rb_head + offset) % rb_len; 
    
    // If the read wraps around the end of the buffer
    if (start_idx + length < rb_len) {
        memcpy(out_ptr, &rb_buffer[start_idx], length * sizeof(float));
    } else {
        // Copy part 1 (end of buffer)
        size_t part1 = rb_len - start_idx;
        memcpy(out_ptr, &rb_buffer[start_idx], part1 * sizeof(float));
        // Copy part 2 (start of buffer)
        memcpy(out_ptr + part1, &rb_buffer[0], (length - part1) * sizeof(float));
    }
    return 0;
} 

void ai_task(void *arg) {
    audio_chunk_t chunk;
    bool trigger_active = false;
    int post_trigger_samples = 0;
    
    // --- COOLDOWN VARS ---
    int trigger_cooldown = 0;
    const int COOLDOWN_SAMPLES = 24000; // 1.5 seconds @ 16kHz
    // ---------------------

    ESP_LOGI(TAG, "AI Task: Ring Buffer + Cooldown Mode Started");

    while (1) {
        if (xQueueReceive(audio_ai_queue, &chunk, portMAX_DELAY) == pdTRUE) {
            
            // 1. ALWAYS WRITE TO RING BUFFER
            for (int i = 0; i < chunk.count; i++) {
                rb_buffer[rb_head] = (float)chunk.samples[i];
                rb_head++;
                if (rb_head >= AI_BUFFER_SIZE) rb_head = 0; 
            }

            // 2. DECREMENT COOLDOWN (If active)
            if (trigger_cooldown > 0) {
                trigger_cooldown -= chunk.count;
                // If we are in cooldown, we skip the trigger check below
                continue; 
            }

            // 3. CHECK FOR TRIGGER
            if (!trigger_active) {
                for (int i = 0; i < chunk.count; i++) {
                    // Threshold > 2000 is good for this WAV file
                    if (abs(chunk.samples[i]) > 2000) { 
                        ESP_LOGW(TAG, ">>> TRIGGER! Level: %d <<<", chunk.samples[i]);
                        trigger_active = true;
                        
                        // 50/50 SPLIT (Your specific request)
                        // Capture 0.5s AFTER the trigger to center the Pop
                        post_trigger_samples = 16000 * 0.5; 
                        break; 
                    }
                }
            }

            // 4. HANDLE ACTIVE TRIGGER
            if (trigger_active) {
                post_trigger_samples -= chunk.count;

                // Once we have recorded the 0.5s tail...
                if (post_trigger_samples <= 0) {
                    
                    ESP_LOGI(TAG, "Running AI on captured history...");
                    
                    signal_t signal;
                    signal.total_length = AI_BUFFER_SIZE;
                    signal.get_data = &ring_buffer_get_data; 

                    ei_impulse_result_t result = { 0 };
                    
                    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);

                    if (r == EI_IMPULSE_OK) {
                         float gunshot_score = 0.0f;
                         for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                            if (strcmp(result.classification[ix].label, "gunshot") == 0) {
                                gunshot_score = result.classification[ix].value;
                            }
                        }
                        
                        ESP_LOGI(TAG, "AI RESULT: %.2f", gunshot_score);
                        
                        if (gunshot_score > CONFIDENCE_THRESHOLD) {
                             ESP_LOGE(TAG, "!!! CONFIRMED GUNSHOT !!!");
                        }
                    }

                    // RESET AND START COOLDOWN
                    trigger_active = false;
                    trigger_cooldown = COOLDOWN_SAMPLES; // Go blind for 1.5s
                    ESP_LOGI(TAG, "Cooldown started... ignoring echoes.");
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
                ESP_LOGI(TAG, "Starting Multi-Task System...");
                
                // Flush Queues before starting
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
    ESP_LOGI(TAG, "System Init - Multi-Tasking Architecture");

    gpio_reset_pin(PIN_LED); gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_BUTTON); gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON, GPIO_PULLUP_ONLY);

    // 1. Create Queues
    audio_net_queue = xQueueCreate(QUEUE_LENGTH, sizeof(audio_chunk_t));
    audio_ai_queue  = xQueueCreate(QUEUE_LENGTH, sizeof(audio_chunk_t));

    if (audio_net_queue == NULL || audio_ai_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queues!");
        return;
    }

    // 2. Init Hardware
    eth_init();
    mic_init();

    // 3. Launch Tasks (Note the Priorities!)
    // Mic Task: Highest Priority (Core 0) -> Keeps I2S buffer empty
    xTaskCreatePinnedToCore(mic_task, "MicTask", 4096, NULL, 10, NULL, 0);

    // Net Task: Medium Priority (Core 0) -> Sends UDP
    xTaskCreatePinnedToCore(net_task, "NetTask", 4096, NULL, 5, NULL, 0);

    // AI Task: Low Priority (Core 1) -> Crunches numbers in background
    // Important: Put AI on Core 1 to separate it from Wifi/Ethernet/Mic interrupts if possible
    xTaskCreatePinnedToCore(ai_task, "AiTask", 8192, NULL, 1, NULL, 1);

    // UI Task: Lowest Priority
    xTaskCreatePinnedToCore(ui_task, "UiTask", 2048, NULL, 1, NULL, 0);

    current_state = APP_STATE_IDLE;
    ESP_LOGI(TAG, "System Ready.");
}