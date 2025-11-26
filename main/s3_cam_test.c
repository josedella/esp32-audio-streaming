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

#define W5500_MR      0x0000   // Mode Register
#define W5500_GAR     0x0001   // Gateway
#define W5500_SUBR    0x0005   // Subnet
#define W5500_SHAR    0x0009   // MAC address
#define W5500_SIPR    0x000F   // Source IP

#define W5500_S0_MR   0x4000   // Socket 0 - Mode
#define W5500_S0_CR   0x4001   // Socket 0 - Command
#define W5500_S0_SR   0x4003   // Socket 0 - Status
#define W5500_S0_PORT 0x4004   // Socket 0 - Source Port


#define SAMPLE_RATE       16000
#define BUFFER_SAMPLES    64

static volatile bool mic_enabled = false;
static int32_t mic_buffer[BUFFER_SAMPLES];

// ------------------------------------------------------
// Microphone task
// ------------------------------------------------------
void mic_task(void *arg)
{
    ESP_LOGI(TAG, "Mic task running");
    int log_counter = 0; // Counter to slow down logs

    while (1)
    {
        if (!mic_enabled) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        size_t bytes_read = 0;
        esp_err_t ret = i2s_read(
            I2S_NUM_0,
            mic_buffer,
            sizeof(mic_buffer),
            &bytes_read,
            portMAX_DELAY
        );

        if (ret == ESP_OK && bytes_read > 0)
        {
            int n = bytes_read / sizeof(int32_t);
            // ... (RMS calculation code remains the same) ...
            
            double acc = 0.0;
            for (int i = 0; i < n; i++) {
                int32_t raw = mic_buffer[i];
                int32_t sample = raw >> 8; 
                if (sample < 0) sample = -sample;
                acc += (double)sample * (double)sample;
            }
            double rms_int = sqrt((double)acc / n);

            // --- FIX: ONLY LOG ONCE EVERY 100 LOOPS ---
            log_counter++;
            // Change 100 to 10
            if (log_counter >= 10) {  // Prints every ~40ms (Much smoother!)
                ESP_LOGI("APP", "RMS: %.2f", rms_int);
                log_counter = 0;
            }
            // ------------------------------------------
        }
        
        // Small delay to yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }
}
// ------------------------------------------------------
// W5500 driver 
// ------------------------------------------------------
static spi_device_handle_t w5500;

void w5500_write_reg(uint16_t addr, uint8_t control, uint8_t data)
{
    uint8_t tx[3 + 1];
    tx[0] = (addr >> 8) & 0xFF;
    tx[1] = addr & 0xFF;
    tx[2] = control;      // Write, 1 byte
    tx[3] = data;

    spi_transaction_t t = {
        .length = 8 * sizeof(tx),
        .tx_buffer = tx
    };
    spi_device_transmit(w5500, &t);
}
 
uint8_t w5500_read_reg(uint16_t addr, uint8_t control)
{
    // We need to send 3 bytes of header + 1 dummy byte to clock in the data
    uint8_t tx[4] = {
        (addr >> 8) & 0xFF, // Address High
        addr & 0xFF,        // Address Low
        control,            // Control Byte
        0x00                // Dummy Byte (to keep clock running for read)
    };
    uint8_t rx[4] = {0};    // Buffer to store received data (we ignore first 3)

    spi_transaction_t t = {
        .length = 8 * 4,    // Total transaction length: 32 bits (4 bytes)
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    
    spi_device_transmit(w5500, &t);

    // The W5500 returns data during the 4th byte (index 3)
    return rx[3];
}

static void w5500_network_init(void)
{
    // Example: your ESP32 = 192.168.1.50
    uint8_t gateway[4]   = {192,168,1,1};
    uint8_t subnet[4]    = {255,255,255,0};
    uint8_t ip[4]        = {192,168,1,50};
    uint8_t mac[6]       = {0x00,0x08,0xDC,0x01,0x02,0x03};

    for(int i=0;i<4;i++)
        w5500_write_reg(0x0001 + i, 0x04, gateway[i]);   // GAR

    for(int i=0;i<4;i++)
        w5500_write_reg(0x0005 + i, 0x04, subnet[i]);    // SUBR

    for(int i=0;i<4;i++)
        w5500_write_reg(0x000F + i, 0x04, ip[i]);        // SIPR

    for(int i=0;i<6;i++)
        w5500_write_reg(0x0009 + i, 0x04, mac[i]);       // SHAR
}

static void w5500_udp_open(void)
{
    w5500_write_reg(0x4000 + 0x0000, 0x0C, 0x02); // Sn_MR = UDP
    w5500_write_reg(0x4000 + 0x0004, 0x0C, 0x0F); // Sn_PORT = 0x0F A0 = 4000 decimal, low byte next
    w5500_write_reg(0x4000 + 0x0005, 0x0C, 0xA0);

    w5500_write_reg(0x4000 + 0x0001, 0x0C, 0x01); // Sn_CR = OPEN
}

void w5500_udp_send_test(void)
{
    uint8_t dest_ip[4] = {192,168,1,100};
    uint16_t dest_port = 5000;
    char msg[] = "PING from ESP32 W5500";

    // Write destination IP
    for(int i=0;i<4;i++)
        w5500_write_reg(0x4000 + 0x000C + i, 0x0C, dest_ip[i]);

    // Write destination port
    w5500_write_reg(0x4000 + 0x0010, 0x0C, dest_port >> 8);
    w5500_write_reg(0x4000 + 0x0011, 0x0C, dest_port & 0xFF);

    // Send data
    for (int i = 0; i < sizeof(msg); i++)
        w5500_write_reg(0x4000 + 0x0020 + i, 0x14, msg[i]);  // TX buffer

    // Issue SEND command
    w5500_write_reg(0x4000 + 0x0001, 0x0C, 0x20);
}

static void w5500_dump_network_regs(void)
{
    ESP_LOGI(TAG, "W5500: Reading back network configuration");

    uint8_t mr  = w5500_read_reg(W5500_MR, 0x00);
    ESP_LOGI(TAG, " MR      = 0x%02X", mr);

    uint8_t gar[4], subr[4], sip[4], mac[6];

    for(int i=0;i<4;i++) gar[i]  = w5500_read_reg(W5500_GAR + i,  0x00);
    for(int i=0;i<4;i++) subr[i] = w5500_read_reg(W5500_SUBR + i, 0x00);
    for(int i=0;i<4;i++) sip[i]  = w5500_read_reg(W5500_SIPR + i, 0x00);
    for(int i=0;i<6;i++) mac[i]  = w5500_read_reg(W5500_SHAR + i, 0x00);

    ESP_LOGI(TAG, " GAR     = %d.%d.%d.%d",
        gar[0], gar[1], gar[2], gar[3]);

    ESP_LOGI(TAG, " SUBR    = %d.%d.%d.%d",
        subr[0], subr[1], subr[2], subr[3]);

    ESP_LOGI(TAG, " SIPR    = %d.%d.%d.%d",
        sip[0], sip[1], sip[2], sip[3]);

    ESP_LOGI(TAG, " SHAR    = %02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void w5500_check_socket(void)
{
    uint8_t mode = w5500_read_reg(W5500_S0_MR, 0x08);
    uint8_t status = w5500_read_reg(W5500_S0_SR, 0x08);
    uint8_t p1 = w5500_read_reg(W5500_S0_PORT + 0, 0x08);
    uint8_t p2 = w5500_read_reg(W5500_S0_PORT + 1, 0x08);
    uint16_t port = (p1 << 8) | p2;

    ESP_LOGI(TAG, "Socket0 MR    = 0x%02X", mode);
    ESP_LOGI(TAG, "Socket0 SR    = 0x%02X", status);
    ESP_LOGI(TAG, "Socket0 PORT  = %u", port);

    if (status == 0x13)
        ESP_LOGI(TAG, "Socket0 Status: UDP OPEN (0x13)");
    else
        ESP_LOGW(TAG, "Socket0 Status unexpected: 0x%02X", status);
}

// ------------------------------------------------------
// Init W5500 SPI Bus
// ------------------------------------------------------
static void init_w5500_spi(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = W5500_MOSI,
        .miso_io_num = W5500_MISO,
        .sclk_io_num = W5500_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2048
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000,   // safe speed (2 MHz)
        .mode = 0,                           // W5500 usually needs mode 0
        .spics_io_num = W5500_CS,            // must NOT be GPIO9
        .queue_size = 4
    };

    spi_device_handle_t handle;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &handle));
    w5500 = handle;

    // Hardware reset
    gpio_reset_pin(W5500_RST);
    gpio_set_direction(W5500_RST, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "W5500 reset start");
    gpio_set_level(W5500_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_set_level(W5500_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));    // important for PHY startup

    // Test read MR register
    uint8_t mr1 = w5500_read_reg(0x0000, 0x00);
    ESP_LOGI(TAG, "MR after reset read = 0x%02X", mr1);

    ESP_LOGI(TAG, "W5500 SPI initialized + reset");
}




// ------------------------------------------------------
// Init I2S for INMP441
// ------------------------------------------------------
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

void udp_ping_task(void *arg)
{
    while (1)
    {
        w5500_udp_send_test();   // sends one packet
        vTaskDelay(pdMS_TO_TICKS(1000));   // 1 second ping interval
    }
}

// ------------------------------------------------------
// Main app
// ------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Starting application");

    // LED
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // Button
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_cfg);

    // Drivers
    init_i2s();
    init_w5500_spi();
    w5500_network_init();
    w5500_dump_network_regs();   // read back registers
    w5500_udp_open();
    w5500_check_socket();

    xTaskCreatePinnedToCore(
        udp_ping_task,
        "udp_ping",
        4096,
        NULL,
        4,
        NULL,
        1
    );


    // Start microphone task
    xTaskCreatePinnedToCore(
        mic_task,
        "mic_task",
        8192,
        NULL,
        5,
        NULL,
        1
    );

    bool led_state = false;
    bool last_button = true;

    ESP_LOGI(TAG, "Press button to toggle LED and microphone");

    while (1)
    {
        bool now = gpio_get_level(BUTTON_PIN);

        if (last_button && !now)
        {
            led_state = !led_state;
            gpio_set_level(LED_PIN, led_state);
            mic_enabled = led_state;

            ESP_LOGI(TAG, "Button toggled. LED %s. Mic %s",
                led_state ? "on" : "off",
                mic_enabled ? "enabled" : "disabled");

            vTaskDelay(pdMS_TO_TICKS(250));
        }

        last_button = now;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
