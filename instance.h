#include "deca_types.h"
#include "deca_device_api.h"

#define LED_RX GPIO_DIR_GDP2_BIT_MASK
#define LED_TX GPIO_DIR_GDP3_BIT_MASK
#define PORT_DE GPIO_DIR_GDP1_BIT_MASK

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y)),



typedef enum Role{
	ROLE_INIT,
	ROLE_RESP,
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

