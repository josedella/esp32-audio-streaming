#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

// --- HARDWARE PINS ---
// Network (W5500)
#define PIN_ETH_MISO    GPIO_NUM_5
#define PIN_ETH_MOSI    GPIO_NUM_6
#define PIN_ETH_SCLK    GPIO_NUM_7
#define PIN_ETH_CS      GPIO_NUM_10
#define PIN_ETH_RST     GPIO_NUM_2

// Audio (INMP441)
#define PIN_I2S_BCLK    GPIO_NUM_47
#define PIN_I2S_WS      GPIO_NUM_21
#define PIN_I2S_DATA    GPIO_NUM_48

// User Interface
#define PIN_LED         GPIO_NUM_4
#define PIN_BUTTON      GPIO_NUM_15

// --- AUDIO SETTINGS ---
#define SAMPLE_RATE     16000
#define BUFFER_SAMPLES  256  // Stereo Frames (L+R)
#define AUDIO_GAIN      14   // Bit shift (>> 14)

// --- NETWORK SETTINGS ---
#define UDP_PORT        5000
// IP Config is handled in w5500_driver, but could be defined here
#endif