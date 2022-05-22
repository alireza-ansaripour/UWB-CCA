#include "deca_types.h"
#include "deca_device_api.h"

typedef struct{
	uint16_t packet_id; // PACKET_ID
	uint8_t sequence_number __attribute__((packed));
	uint16_t pan_id; __attribute__((packed)); // PAN_ID
	uint16_t dst;
	uint16_t src;
	uint16_t crc;
        uint8_t payload[900];
} packet_std_t __attribute__((packed));

typedef  struct{
  uint32_t sequence_number;
}packet_info_t __attribute__((packed)); 

#define MSG_TIME_SYNC					0x76
#define MSG_DATA                                        0xDA


#define LED_RX GPIO_DIR_GDP2_BIT_MASK
#define LED_TX GPIO_DIR_GDP3_BIT_MASK
#define PORT_DE GPIO_DIR_GDP1_BIT_MASK

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y)),



typedef enum Role{
	ROLE_TX,
	ROLE_RX,
	ROLE_TS,
        ROLE_TX_TS,
}Role;

typedef enum Radio_action{
  ACTION_TX,
  ACTION_TX_DLY,
  ACTION_RX,
  ACTION_NONE,
}Radio_action; 

//typedef enum { STATE_RX, STATE_TX } state_machine_state_t;
typedef struct
{
	uint8_t enabled;
	uint32_t timeout;
} event_t;

typedef struct
{
	uint8_t enabled;
	uint32_t timeout;
	uint16_t packet_size;
} event_data_t;





void instance_init();
void instance_loop();
void send_UART_msg(uint8_t *msg, uint16_t payload_len);

