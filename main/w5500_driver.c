#include "w5500_driver.h"
#include "config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "ETH";
static spi_device_handle_t w5500; // Now it will be used below!
static uint16_t seq_num = 0;
static uint32_t timestamp = 0;

// --- INTERNAL HELPER FUNCTIONS ---
// (Copied from your old main)
static void w5500_write_reg(uint16_t addr, uint8_t control, uint8_t data) {
    uint8_t tx[4] = { (addr >> 8) & 0xFF, addr & 0xFF, control, data };
    spi_transaction_t t = { .length = 32, .tx_buffer = tx };
    spi_device_transmit(w5500, &t);
}

static uint8_t w5500_read_reg(uint16_t addr, uint8_t control) {
    uint8_t tx[4] = { (addr >> 8) & 0xFF, addr & 0xFF, control, 0x00 };
    uint8_t rx[4] = {0};
    spi_transaction_t t = { .length = 32, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_transmit(w5500, &t);
    return rx[3];
}

static uint16_t w5500_read_reg16(uint16_t addr, uint8_t control) {
    uint16_t res = w5500_read_reg(addr, control) << 8;
    res |= w5500_read_reg(addr + 1, control);
    return res;
}

static void w5500_write_reg16(uint16_t addr, uint8_t control, uint16_t data) {
    w5500_write_reg(addr, control, (data >> 8) & 0xFF);
    w5500_write_reg(addr + 1, control, data & 0xFF);
}

static void w5500_write_block(uint16_t addr, uint8_t control, uint8_t *data, size_t len) {
    uint8_t *tx_buf = heap_caps_malloc(3 + len, MALLOC_CAP_DMA);
    if (!tx_buf) return;

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

// --- PUBLIC FUNCTIONS ---

void eth_init(void) {
    ESP_LOGI(TAG, "Initializing SPI...");
    
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_ETH_MOSI,
        .miso_io_num = PIN_ETH_MISO,
        .sclk_io_num = PIN_ETH_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    // Note: SPI2_HOST is standard for S3
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 16 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_ETH_CS,
        .queue_size = 4
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &w5500));

    // Hardware Reset
    gpio_reset_pin(PIN_ETH_RST);
    gpio_set_direction(PIN_ETH_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_ETH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_set_level(PIN_ETH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Initial Config
    uint8_t gateway[4] = {192,168,1,1};
    uint8_t subnet[4]  = {255,255,255,0};
    uint8_t ip[4]      = {192,168,1,50};
    uint8_t mac[6]     = {0x00,0x08,0xDC,0x01,0x02,0x03};

    // Using hardcoded register offsets (from datasheet)
    for(int i=0;i<4;i++) w5500_write_reg(0x0001 + i, 0x04, gateway[i]);
    for(int i=0;i<4;i++) w5500_write_reg(0x0005 + i, 0x04, subnet[i]);
    for(int i=0;i<4;i++) w5500_write_reg(0x000F + i, 0x04, ip[i]);
    for(int i=0;i<6;i++) w5500_write_reg(0x0009 + i, 0x04, mac[i]);

    // Open UDP Socket (Socket 0)
    w5500_write_reg(0x4000, 0x0C, 0x02); // Sn_MR = UDP
    w5500_write_reg(0x4004, 0x0C, 0x0F); // Port 4000 (High)
    w5500_write_reg(0x4005, 0x0C, 0xA0); // Port 4000 (Low)
    w5500_write_reg(0x4001, 0x0C, 0x01); // OPEN

    // Set Destination IP/Port (Your Laptop)
    // IP: 192.168.1.23, Port: 5000
    uint8_t dest_ip[4] = {192, 168, 1, 138};
    for(int i=0; i<4; i++) w5500_write_reg(0x400C + i, 0x0C, dest_ip[i]);
    w5500_write_reg(0x4010, 0x0C, 5000 >> 8);
    w5500_write_reg(0x4011, 0x0C, 5000 & 0xFF);

    ESP_LOGI(TAG, "Ethernet Init Complete");
}

void eth_send_rtp_packet(int16_t *pcm_data, size_t sample_count) {
    size_t payload_len = sizeof(rtp_header_t) + (sample_count * sizeof(int16_t));
    uint8_t *packet = heap_caps_malloc(payload_len, MALLOC_CAP_8BIT);
    
    if (packet) {
        // 1. Build RTP Header
        rtp_header_t *rtp = (rtp_header_t *)packet;
        rtp->version = 0x80;
        rtp->payloadType = 96;
        
        // FIX: Safe increment
        uint16_t current_seq = seq_num;
        seq_num++;
        rtp->sequence = (current_seq << 8) | (current_seq >> 8);
        
        // Timestamp (Big Endian Swap)
        rtp->timestamp = ((timestamp & 0xFF) << 24) | ((timestamp & 0xFF00) << 8) | 
                         ((timestamp & 0xFF0000) >> 8) | ((timestamp >> 24) & 0xFF);
        
        rtp->ssrc = 0x12345678; // Constant ID
        timestamp += sample_count;

        // 2. Copy Audio
        memcpy(packet + sizeof(rtp_header_t), pcm_data, sample_count * sizeof(int16_t));

        // 3. Send to W5500 Ring Buffer
        uint16_t ptr = w5500_read_reg16(0x4024, 0x08);
        uint16_t offset = ptr & 0x07FF;
        uint16_t free_space = 2048 - offset;

        if (payload_len <= free_space) {
            w5500_write_block(offset, 0x14, packet, payload_len);
        } else {
            w5500_write_block(offset, 0x14, packet, free_space);
            w5500_write_block(0x0000, 0x14, packet + free_space, payload_len - free_space);
        }

        ptr += payload_len;
        w5500_write_reg16(0x4024, 0x0C, ptr);
        w5500_write_reg(0x4001, 0x0C, 0x20); // SEND Command
        w5500_write_reg(0x4002, 0x0C, 0x10); // Clear Interrupt

        free(packet);
    }
}

// --- NEW: Add this helper to read back configuration ---
void eth_verify_config(void) {
    uint8_t ip[4];
    // Read Source IP Register (SIPR) at 0x000F, Block 0x00
    for(int i=0; i<4; i++) {
        ip[i] = w5500_read_reg(0x000F + i, 0x00);
    }
    
    ESP_LOGI(TAG, "W5500 Configured IP: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    if (ip[0] == 0 && ip[1] == 0) {
        ESP_LOGE(TAG, "CRITICAL: W5500 IP is 0.0.0.0! SPI Communication Failed.");
    } else {
        ESP_LOGI(TAG, "SPI Communication OK.");
    }
}

// --- IMPLEMENTATION of eth_check_link ---
bool eth_check_link(void) {
    // PHY Configuration Register (PHYCFGR) is at address 0x002E in Common Block (0x00)
    // Bit 0 represents the Link Status (1 = Link Up, 0 = Link Down)
    uint8_t phy_status = w5500_read_reg(0x002E, 0x00);
    
    // Check Bit 0
    if (phy_status & 0x01) {
        return true; // Link Up
    }
    return false; // Link Down
}