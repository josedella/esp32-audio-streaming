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

// LED and button (Moved Button to GPIO 15 for better layout)
#define LED_PIN        GPIO_NUM_4
#define BUTTON_PIN     GPIO_NUM_15 

// INMP441 I2S microphones (Stereo)
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
#define BUFFER_SAMPLES    256   // Increased from 64 to reduce network jitter!

// Globals
static volatile bool mic_enabled = false;
// Buffer size needs to handle stereo (2 channels * 32 bits)
static int32_t mic_buffer[BUFFER_SAMPLES * 2]; 
static spi_device_handle_t w5500;

// ------------------------------------------------------
// W5500 Low-Level Drivers (Read/Write)
// ------------------------------------------------------

void w5500_write_reg(uint16_t addr, uint8_t control, uint8_t data)
{
    uint8_t tx[4];
    tx[0] = (addr >> 8) & 0xFF;
    tx[1] = addr & 0xFF;
    tx[2] = control;      
    tx[3] = data;

    spi_transaction_t t = {
        .length = 8 * 4,
        .tx_buffer = tx
    };
    spi_device_transmit(w5500, &t);
}

uint8_t w5500_read_reg(uint16_t addr, uint8_t control)
{
    uint8_t tx[4] = {
        (addr >> 8) & 0xFF, 
        addr & 0xFF,        
        control,            
        0x00                
    };
    uint8_t rx[4] = {0};    

    spi_transaction_t t = {
        .length = 8 * 4,    
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    
    spi_device_transmit(w5500, &t);
    return rx[3];
}

// Helper to read 16-bit register 
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

// Helper to write a BLOCK of data to W5500 (Efficient!)
void w5500_write_block(uint16_t addr, uint8_t control, uint8_t *data, size_t len)
{
    // We need to send 3 bytes (Addr/Ctrl) + Data Length
    // We can't just allocate a huge array on stack, so we do it in chunks or use a smart transaction
    // Better approach for ESP-IDF: Create a transaction with `tx_buffer` pointing to the data
    // BUT, W5500 needs Addr/Ctrl FIRST. 
    
    // Strategy: Send Addr/Ctrl, then keep CS low and send Data.
    // ESP-IDF 'spi_device_transmit' handles CS automatically. 
    // We will create a temporary buffer for the Whole Frame (Header + Data)
    
    // Max packet is ~512 bytes, so this is safe on stack or heap.
    uint8_t *tx_buf = heap_caps_malloc(3 + len, MALLOC_CAP_DMA);
    if (!tx_buf) return; // Error handling

    tx_buf[0] = (addr >> 8) & 0xFF;
    tx_buf[1] = addr & 0xFF;
    tx_buf[2] = control;
    memcpy(&tx_buf[3], data, len);

    spi_transaction_t t = {
        .length = 8 * (3 + len),
        .tx_buffer = tx_buf
    };
    
    spi_device_transmit(w5500, &t);
    free(tx_buf);
}

// Optimized UDP Send Function
void w5500_send_udp_packet(uint8_t *data, size_t len) {
    // 1. Get current Write Pointer
    uint16_t ptr = w5500_read_reg16(0x4024, 0x08);
    
    // 2. Write data to Circular Buffer (Handling Wrap-around)
    uint16_t offset = ptr & 0x07FF; // Mask for 2KB buffer
    uint16_t free_space_at_end = 2048 - offset;

    if (len <= free_space_at_end) {
        // Easy case: No wrap-around, just write it all
        w5500_write_block(offset, 0x14, data, len);
    } else {
        // Hard case: Wrap-around needed
        // Write first part (to end of buffer)
        w5500_write_block(offset, 0x14, data, free_space_at_end);
        // Write second part (from start of buffer)
        w5500_write_block(0x0000, 0x14, data + free_space_at_end, len - free_space_at_end);
    }

    // 3. Update Write Pointer
    ptr += len;
    w5500_write_reg16(0x4024, 0x0C, ptr); 

    // 4. Issue SEND command
    w5500_write_reg(W5500_S0_CR, 0x0C, 0x20); 

    // 5. Clear Interrupts
    w5500_write_reg(0x4002, 0x0C, 0x10); 
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
    if (status == 0x22) 
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
        .clock_speed_hz = 16 * 1000 * 1000, 
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
        // CHANGED: Use Right and Left channels (Stereo)
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
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
    gpio_set_drive_capability(I2S_BCLK_PIN, GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(I2S_WS_PIN, GPIO_DRIVE_CAP_0);
    ESP_LOGI(TAG, "I2S initialized (Stereo)");
}

// ------------------------------------------------------
// Microphone Task (Audio Streamer)
// ------------------------------------------------------

// TARGET: YOUR LAPTOP IP
uint8_t dest_ip[4] = {192, 168, 1, 138}; 
uint16_t dest_port = 5000;

// --- RTP Protocol Definitions ---
// __attribute__((packed)) prevents the compiler from adding gaps/padding
typedef struct __attribute__((packed)) {
    uint8_t version;      
    uint8_t payloadType;  
    uint16_t sequence;    
    uint32_t timestamp;   
    uint32_t ssrc;        
} rtp_header_t;

// Helper to swap Endianness
uint16_t htons(uint16_t v) { return (v << 8) | (v >> 8); }
uint32_t htonl(uint32_t v) {
    return ((v & 0xFF) << 24) | ((v & 0xFF00) << 8) | 
           ((v & 0xFF0000) >> 8) | ((v >> 24) & 0xFF);
}

void mic_task(void *arg)
{
    ESP_LOGI(TAG, "Mic task running - RTP MODE");

    // Configure W5500
    for(int i=0; i<4; i++) w5500_write_reg(0x400C + i, 0x0C, dest_ip[i]);
    w5500_write_reg(0x4010, 0x0C, dest_port >> 8);
    w5500_write_reg(0x4011, 0x0C, dest_port & 0xFF);

    uint16_t seq_num = 0;
    uint32_t timestamp = 0;
    uint32_t ssrc_id = 0x12345678; 

    while (1)
    {
        if (!mic_enabled) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        size_t bytes_read = 0;
        esp_err_t ret = i2s_read(I2S_NUM_0, mic_buffer, sizeof(mic_buffer), &bytes_read, portMAX_DELAY);

        if (ret == ESP_OK && bytes_read > 0)
        {
            int samples = bytes_read / sizeof(int32_t);
            
            // ⚠️ CRITICAL: Payload size = 12 byte Header + Audio Data
            size_t payload_size = sizeof(rtp_header_t) + (samples * sizeof(int16_t));
            uint8_t udp_packet[payload_size];

            // 1. Fill RTP Header (12 Bytes)
            rtp_header_t *rtp = (rtp_header_t *)udp_packet;
            rtp->version = 0x80;
            rtp->payloadType = 96;
            rtp->sequence = htons(seq_num++);
            rtp->timestamp = htonl(timestamp);
            rtp->ssrc = htonl(ssrc_id);
            timestamp += samples;

            // 2. Fill Audio Data (Correctly Aligned)
            int16_t *pcm_ptr = (int16_t *)(udp_packet + sizeof(rtp_header_t));
            
            for (int i = 0; i < samples; i++)
            {
                int32_t raw = mic_buffer[i];
                // Use >> 14 for safe gain (>> 12 was clipping in your CSV)
                int32_t amplified = raw >> 14; 
                
                if (amplified > 32767) amplified = 32767;
                if (amplified < -32768) amplified = -32768;
                
                pcm_ptr[i] = (int16_t)amplified; 
            }
            // Add this temporary debug line:
            //ESP_LOGI(TAG, "Payload Size: %d (Expected: %d)", payload_size, 12 + (samples * 2));
            // 3. Send
            w5500_send_udp_packet(udp_packet, payload_size);
        }
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