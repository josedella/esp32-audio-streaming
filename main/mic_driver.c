#include "mic_driver.h"
#include "config.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "MIC";

void mic_init(void) {
    ESP_LOGI(TAG, "Initializing I2S...");

    i2s_config_t i2s_cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stereo
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 128,
        .use_apll = false
    };

    i2s_pin_config_t pin_cfg = {
        .bck_io_num = PIN_I2S_BCLK,
        .ws_io_num = PIN_I2S_WS,
        .data_out_num = -1,
        .data_in_num = PIN_I2S_DATA
    };

    // 1. Install Driver
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2S driver: %s", esp_err_to_name(err));
        return;
    }

    // 2. Set Pins
    err = i2s_set_pin(I2S_NUM_0, &pin_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2S pins: %s", esp_err_to_name(err));
        return;
    }

    // 3. Clear Buffer
    i2s_zero_dma_buffer(I2S_NUM_0);

    // 4. Reduce Noise (Optional but recommended)
    gpio_set_drive_capability(PIN_I2S_BCLK, GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(PIN_I2S_WS, GPIO_DRIVE_CAP_0);

    ESP_LOGI(TAG, "I2S Initialized (Stereo)");
}

size_t mic_read_stereo(int32_t *raw_buffer, size_t buffer_len) {
    size_t bytes_read = 0;
    // Read from I2S (buffer_len is number of 32-bit samples)
    i2s_read(I2S_NUM_0, raw_buffer, buffer_len * sizeof(int32_t), &bytes_read, portMAX_DELAY);
    return bytes_read / sizeof(int32_t); // Return number of samples read
}