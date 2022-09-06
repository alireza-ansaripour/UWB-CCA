#include "deca_types.h"

#define TWR_POLL    0x01
#define TWR_RESP    0x02
#define TWR_FINAL   0x03


typedef struct __attribute__((packed)){
  uint16_t packet_id; // PACKET_ID
  uint8_t sequence_number __attribute__((packed));
  uint16_t pan_id; __attribute__((packed)); // PAN_ID
  uint16_t dst;
  uint16_t src;
  uint8_t msg_type;
  uint8_t payload[900];
} packet_std_t; 

#define PACKET_STD_HDR_LEN 10

typedef  struct __attribute__((packed)){
  uint32_t sequence_number;
  uint8_t payload[900];
}packet_data_t ; 

typedef struct __attribute__((packed)){
  uint32_t poll_tx_ts;
  uint32_t final_tx_ts;
  uint32_t resp_rx_ts[2];
} packet_ranging_t; 


typedef struct{
  uint32_t sequence_number;
  uint32_t N;
  uint16_t C;
  uint16_t dcg;
}report_packet_t __attribute__((packed));
