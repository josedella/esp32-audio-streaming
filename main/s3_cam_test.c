#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string.h>
#include <math.h>

static const char *TAG = "APP";

// ------------------------------------------------------
// GPIO assignments
// ------------------------------------------------------

// LED and button
#define LED_PIN        GPIO_NUM_4
#define BUTTON_PIN     GPIO_NUM_42

// INMP441 I2S microphone
#define I2S_BCLK_PIN   GPIO_NUM_47
#define I2S_WS_PIN     GPIO_NUM_21
#define I2S_DATA_PIN   GPIO_NUM_48

// W5500 SPI Ethernet
#define W5500_MISO     GPIO_NUM_5
#define W5500_MOSI     GPIO_NUM_6
#define W5500_SCK      GPIO_NUM_7
#define W5500_CS       GPIO_NUM_10     
#define W5500_RST      GPIO_NUM_2

// W5500 Register Definitions
#define W5500_MR       0x0000   // Mode Register
#define W5500_GAR      0x0001   // Gateway
#define W5500_SUBR     0x0005   // Subnet
#define W5500_SHAR     0x0009   // MAC address
#define W5500_SIPR     0x000F   // Source IP

#define W5500_S0_MR    0x4000   // Socket 0 - Mode
#define W5500_S0_CR    0x4001   // Socket 0 - Command
#define W5500_S0_SR    0x4003   // Socket 0 - Status
#define W5500_S0_PORT  0x4004   // Socket 0 - Source Port

// Audio Settings
#define SAMPLE_RATE       16000
#define BUFFER_SAMPLES    64

// Globals
static volatile bool mic_enabled = false;
static int32_t mic_buffer[BUFFER_SAMPLES];
static spi_device_handle_t w5500;

// ------------------------------------------------------
// W5500 Low-Level Drivers (Read/Write)
// ------------------------------------------------------

void w5500_write_reg(uint16_t addr, uint8_t control, uint8_t data)
{
    uint8_t tx[4];
    tx[0] = (addr >> 8) & 0xFF;
    tx[1] = addr & 0xFF;
    tx[2] = control;      // Write, 1 byte
    tx[3] = data;

    spi_transaction_t t = {
        .length = 8 * 4,
        .tx_buffer = tx
    };
    spi_device_transmit(w5500, &t);
}

uint8_t w5500_read_reg(uint16_t addr, uint8_t control)
{
    // Send 3 bytes header + 1 dummy byte to clock in data
    uint8_t tx[4] = {
        (addr >> 8) & 0xFF, 
        addr & 0xFF,        
        control,            
        0x00                // Dummy Byte
    };
    uint8_t rx[4] = {0};    

    spi_transaction_t t = {
        .length = 8 * 4,    
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    
    spi_device_transmit(w5500, &t);

    // The W5500 returns data during the 4th byte (index 3)
    return rx[3];
}

// Helper to read 16-bit register (needed for pointers)
uint16_t w5500_read_reg16(uint16_t addr, uint8_t control) {
    uint16_t res = w5500_read_reg(addr, control) << 8;
    res |= w5500_read_reg(addr + 1, control);
    return res;
}

// Helper to write 16-bit register
void w5500_write_reg16(uint16_t addr, uint8_t control, uint16_t data) {
    w5500_write_reg(addr, control, (data >> 8) & 0xFF);
    w5500_write_reg(addr + 1, control, data & 0xFF);
}

// ------------------------------------------------------
// W5500 High-Level Logic
// ------------------------------------------------------

static void w5500_network_init(void)
{
    // ESP32 IP Configuration
    uint8_t gateway[4]   = {192,168,1,1};
    uint8_t subnet[4]    = {255,255,255,0};
    uint8_t ip[4]        = {192,168,1,50};
    uint8_t mac[6]       = {0x00,0x08,0xDC,0x01,0x02,0x03};

    for(int i=0;i<4;i++) w5500_write_reg(W5500_GAR + i, 0x04, gateway[i]);
    for(int i=0;i<4;i++) w5500_write_reg(W5500_SUBR + i, 0x04, subnet[i]);
    for(int i=0;i<4;i++) w5500_write_reg(W5500_SIPR + i, 0x04, ip[i]);
    for(int i=0;i<6;i++) w5500_write_reg(W5500_SHAR + i, 0x04, mac[i]);
}

static void w5500_udp_open(void)
{
    w5500_write_reg(W5500_S0_MR, 0x0C, 0x02); // Sn_MR = UDP
    w5500_write_reg(W5500_S0_PORT, 0x0C, 0x0F); // Port High
    w5500_write_reg(W5500_S0_PORT + 1, 0x0C, 0xA0); // Port Low (4000)
    w5500_write_reg(W5500_S0_CR, 0x0C, 0x01); // Sn_CR = OPEN
}

// Send raw UDP data packet (Socket 0)
void w5500_send_udp_packet(uint8_t *data, size_t len) {
    // 1. Get current Write Pointer (Sn_TX_WR is at offset 0x0024)
    uint16_t ptr = w5500_read_reg16(0x4024, 0x08);

    // 2. Write data to the circular buffer
    for (size_t i = 0; i < len; i++) {
        uint16_t addr = (ptr + i) & 0x07FF; // Mask for 2KB buffer
        w5500_write_reg(addr, 0x14, data[i]); 
    }

    // 3. Update Write Pointer
    ptr += len;
    w5500_write_reg16(0x4024, 0x0C, ptr); 

    // 4. Issue SEND command
    w5500_write_reg(W5500_S0_CR, 0x0C, 0x20); // SEND

    // 5. Clear Interrupts (Optional but good practice)
    w5500_write_reg(0x4002, 0x0C, 0x10); // Clear SEND_OK bit
}

static void w5500_dump_network_regs(void)
{
    ESP_LOGI(TAG, "W5500: Reading back network configuration");
    uint8_t mr  = w5500_read_reg(W5500_MR, 0x00);
    ESP_LOGI(TAG, " MR      = 0x%02X", mr);

    uint8_t ip[4];
    for(int i=0;i<4;i++) ip[i] = w5500_read_reg(W5500_SIPR + i, 0x00);
    ESP_LOGI(TAG, " IP      = %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}

static void w5500_check_socket(void)
{
    uint8_t status = w5500_read_reg(W5500_S0_SR, 0x08);
    if (status == 0x22) // 0x22 is UDP OPEN
        ESP_LOGI(TAG, "Socket0 Status: UDP OPEN (0x22)");
    else
        ESP_LOGW(TAG, "Socket0 Status unexpected: 0x%02X", status);
}

// ------------------------------------------------------
// Hardware Initialization
// ------------------------------------------------------

static void init_w5500_spi(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = W5500_MOSI,
        .miso_io_num = W5500_MISO,
        .sclk_io_num = W5500_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 16 * 1000 * 1000,  // 16 MHz (W5500 handles up to 33MHz usually)
        .mode = 0,
        .spics_io_num = W5500_CS,
        .queue_size = 4
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &w5500));

    // Hardware reset
    gpio_reset_pin(W5500_RST);
    gpio_set_direction(W5500_RST, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "W5500 reset start");
    gpio_set_level(W5500_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_set_level(W5500_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200)); 
}

static void init_i2s(void)
{
    i2s_config_t i2s_cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 128,
        .use_apll = false
    };

    i2s_pin_config_t pin_cfg = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = -1,
        .data_in_num = I2S_DATA_PIN
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_cfg));
    i2s_zero_dma_buffer(I2S_NUM_0);

    ESP_LOGI(TAG, "I2S initialized");
}

// ------------------------------------------------------
// Microphone Task (Audio Streamer)
// ------------------------------------------------------

// TARGET: YOUR LAPTOP IP
uint8_t dest_ip[4] = {192, 168, 1, 23}; 
uint16_t dest_port = 5000;

void mic_task(void *arg)
{
    ESP_LOGI(TAG, "Mic task running");

    // Configure Destination IP & Port for Socket 0
    for(int i=0; i<4; i++) w5500_write_reg(0x400C + i, 0x0C, dest_ip[i]);
    w5500_write_reg(0x4010, 0x0C, dest_port >> 8);
    w5500_write_reg(0x4011, 0x0C, dest_port & 0xFF);

    while (1)
    {
        if (!mic_enabled) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        size_t bytes_read = 0;
        esp_err_t ret = i2s_read(I2S_NUM_0, mic_buffer, sizeof(mic_buffer), &bytes_read, portMAX_DELAY);

        if (ret == ESP_OK && bytes_read > 0)
        {
            int samples = bytes_read / sizeof(int32_t);
            int16_t pcm_output[samples]; 

            for (int i = 0; i < samples; i++)
            {
                int32_t raw = mic_buffer[i];
                // INMP441 24-bit MSB aligned -> 16-bit PCM
                pcm_output[i] = (int16_t)(raw >> 14); 
            }

            // Stream Audio via UDP
            w5500_send_udp_packet((uint8_t*)pcm_output, samples * sizeof(int16_t));
        }
        // No delay: Stream as fast as I2S provides data
    }
}

// ------------------------------------------------------
// Main App
// ------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Audio Streamer");

    // Button & LED Config
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_cfg);

    // Hardware Initialization
    init_i2s();
    init_w5500_spi();
    
    // Network Setup
    w5500_network_init();
    w5500_dump_network_regs();
    w5500_udp_open();
    w5500_check_socket();

    // Start Tasks
    xTaskCreatePinnedToCore(mic_task, "mic_task", 8192, NULL, 5, NULL, 1);

    // Button Loop
    bool led_state = false;
    bool last_button = true;

    ESP_LOGI(TAG, "Ready. Press button to toggle streaming.");

    while (1)
    {
        bool now = gpio_get_level(BUTTON_PIN);

        if (last_button && !now)
        {
            led_state = !led_state;
            gpio_set_level(LED_PIN, led_state);
            mic_enabled = led_state;

            ESP_LOGI(TAG, "Stream %s", led_state ? "STARTED" : "STOPPED");
            vTaskDelay(pdMS_TO_TICKS(250)); // Debounce
        }

        last_button = now;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}