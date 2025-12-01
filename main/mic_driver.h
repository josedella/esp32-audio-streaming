#ifndef MIC_DRIVER_H
#define MIC_DRIVER_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialize the I2S driver for Stereo Audio
 * Configures pins, clock, and data format (32-bit, 16kHz).
 */
void mic_init(void);

/**
 * @brief Read raw stereo audio data from the I2S buffer
 * * @param raw_buffer Pointer to the buffer where 32-bit raw I2S data will be stored
 * @param buffer_len Total number of elements (int32_t) the buffer can hold
 * @return size_t Number of samples actually read
 */
size_t mic_read_stereo(int32_t *raw_buffer, size_t buffer_len);

#endif // MIC_DRIVER_H