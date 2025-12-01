#ifndef W5500_DRIVER_H
#define W5500_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


// RTP Header Definition (Shared)
typedef struct __attribute__((packed)) {
    uint8_t version;      
    uint8_t payloadType;  
    uint16_t sequence;    
    uint32_t timestamp;   
    uint32_t ssrc;        
} rtp_header_t;

void eth_init(void);
void eth_send_rtp_packet(int16_t *pcm_data, size_t sample_count);
bool eth_check_link(void);

#endif