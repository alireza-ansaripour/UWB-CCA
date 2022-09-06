#include "instance.h"
#include <stdio.h>
#include <stdlib.h>
#include <sdk_config.h>
#include "boards.h"
#include "port.h"
#include "deca_spi.h"
#include <math.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port.h"
#include "identity.h"
#include "util.h"
#include "instance_config.h"
#include "ring_buffer.h"
#include "shared_defines.h"
#include "mac_802_15_4.h"
#include "nrf_drv_uart.h"
#include "messages.h"
#include "uwb_messages.h"
#define CIR_SAMPLE 32
#define CIR_SAMPLE_BUFFER CIR_SAMPLE * 6 + 1

int data_index = 0;
uint8_t UART_msg_payload[200];
uint8_t UART_rx_msg_payload[200];
uint8_t UART_byte, packet_len;
static uint64_t m_systick_cnt;
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
static nrf_drv_uart_t app_uart_inst = NRF_DRV_UART_INSTANCE(APP_UART_DRIVER_INSTANCE);
enum State_machine {
  SYNC_1, SYNC_2, P_LEN, RECV_DATA, DATA_DONE
};

void set_dwt_config(dwt_config_t *config){
  memcpy(&instance_info.config.radio_config, config, sizeof(dwt_config_t));
  instance_info.node.update_radio_config = 1;
}

enum State_machine state = SYNC_1;
void handle_uart_message(uint8_t *data, size_t data_len){
  uint8_t msg_type = data[0];
  uint8_t *payload = &data[1];
  uint32_t *seq;
  uint64_t *wait_time;
  switch(msg_type){
    case CONFIG:
      set_dwt_config((dwt_config_t *) payload);
    break;
    case START_TX:
      instance_info.node.tx_enable = 1;
      instance_info.node.tx_timestape = m_systick_cnt;
    break;
    case SEQ_NUM:
      seq = (uint32_t *) payload;
      instance_info.node.sequence_number = *seq;
    break;
    case TX_NUM:
      seq = (uint32_t *) payload;
      instance_info.node.end_sequence_number = instance_info.node.sequence_number + *seq;
    break;
    case WAIT_TIME:
      wait_time = (uint64_t *) payload;
      instance_info.node.tx_delay_ms = wait_time;
    break;

      //instance_info.config.sequence_number = 
  };
  //printf("MSG type: %X\n", msg_type);

}


static void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
    if (p_event->type == NRF_DRV_UART_EVT_RX_DONE)
    {
        if (p_event->data.rxtx.bytes)
        {
          switch (state){
            case SYNC_1:
            if (UART_byte == UART_HEADER[0]){
              state = SYNC_2;
            }
            break;
            case SYNC_2:
              if (UART_byte == UART_HEADER[1])
                state = P_LEN;
              else
                state = SYNC_1;
              break;
            case P_LEN:
              packet_len = UART_byte;
              data_index = 0;
              state = RECV_DATA;
            break;
            case RECV_DATA:
              UART_rx_msg_payload[data_index] = UART_byte;
              data_index++;
            if (data_index == packet_len){
              data_index = 0;
              state = SYNC_1;
              uint8_t msg_type = UART_rx_msg_payload[0];
              UART_ACK[sizeof(UART_ACK) - 1] = msg_type;
              handle_uart_message(UART_rx_msg_payload, packet_len);
              //send_UART_msg(UART_ACK, sizeof(UART_ACK));
            }
          }

          nrf_drv_uart_rx(&app_uart_inst, &UART_byte, 1);

            
            
        }
    }
    else if (p_event->type == NRF_DRV_UART_EVT_ERROR)
    {
      printf("ERR\n");
      nrf_drv_uart_rx(&app_uart_inst, &UART_byte, 1);
        // Event to notify that an error has occured in the UART peripheral
    }
    else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE)
    {
       // Event to notify that the last byte from FIFO has been transmitted

    }
}

static void uart_init(){
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.baudrate = UART_BAUDRATE_BAUDRATE_Baud115200; //User defined
    uart_config.hwfc = NRF_UART_HWFC_DISABLED; //User defined
    uart_config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST; //User defined
    uart_config.parity = NRF_UART_PARITY_EXCLUDED; //User defined
    uart_config.pselcts = CTS_PIN_NUMBER; //User defined. Remove this line if flow control is disabled.
    uart_config.pselrts = RTS_PIN_NUMBER; //User defined. Remove this line if flow control is disabled.
    uart_config.pselrxd = RX_PIN_NUMBER; //User defined
    uart_config.pseltxd = TX_PIN_NUMBER; //User defined

    uint32_t err_code = nrf_drv_uart_init(&app_uart_inst, &uart_config, uart_event_handler);
    APP_ERROR_CHECK(err_code);
}


#define ADDRESS 0xFFFF
#define SET_OUPUT_GPIOs 0xFFFF & ~(GPIO_DIR_GDP1_BIT_MASK | GPIO_DIR_GDP2_BIT_MASK | GPIO_DIR_GDP3_BIT_MASK)
#define ENABLE_ALL_GPIOS_MASK 0x200000
#define SWITCH_CONF_INDEX 10


uint32_t sequence_numbers[10000];
uint8_t rx_buffer[100];
mac_frame_802_15_4_format_t mac_frame;

uint32_t seq_num = 0;
uint32_t tx_num = 0;

uint32_t rx_timestamp;

uint32_t swap_uint32( uint32_t num )
{
    return((num>>24)&0xff) | // move byte 3 to byte 0
          ((num<<8)&0xff0000) | // move byte 1 to byte 2
          ((num>>8)&0xff00) | // move byte 2 to byte 1
          ((num<<24)&0xff000000); // byte 0 to byte 3;
}

uint16_t swap_uint16( uint16_t num )
{
    return (num>>8) | (num<<8);
}


void init_LEDs(){
    dwt_enablegpioclocks();
    dwt_write32bitoffsetreg(GPIO_MODE_ID, 0, ENABLE_ALL_GPIOS_MASK);
    dwt_write16bitoffsetreg(GPIO_OUT_ID, 0, 0x0);
    dwt_write16bitoffsetreg(GPIO_DIR_ID, 0, SET_OUPUT_GPIOs);
}
int res;

void gpio_set(uint16_t port){
	dwt_or16bitoffsetreg(GPIO_OUT_ID, 0, (port));
}

void gpio_reset(uint16_t port){
	dwt_and16bitoffsetreg(GPIO_OUT_ID, 0, (uint16_t)(~(port)));
}


int flg = 0;


void SysTick_Handler(void) {
    m_systick_cnt++;
    //if (m_systick_cnt % 1000 == 0){
    //  if (flg == 0){
    //    gpio_set(LED_RX);
    //  }else{
    //    gpio_reset(LED_RX);
    //  }
    //  flg = (flg + 1) % 2;
      
    //}
}

//void SysTick_Handler(void)  {
//     if(++m_systick_cnt == 500) {
//         LEDS_INVERT(BSP_LED_1_MASK); /* light LED 2 very 1 second */
//         m_systick_cnt = 0;
//     }
// }

void init_NRF(){
  bsp_board_init(BSP_INIT_BUTTONS);
  /* Initialise the SPI for nRF52840-DK */
  nrf52840_dk_spi_init();
  /* Configuring interrupt*/
  dw_irq_init();
  /* Small pause before startup */
  nrf_delay_ms(2);
  SysTick_Config(64);
  NVIC_EnableIRQ(SysTick_IRQn);
}




packet_std_t tx_packet;
packet_ranging_t * ranging_data;

void create_tx_packet(){
  tx_packet.packet_id = 0x8841;
  tx_packet.sequence_number = 0;
  tx_packet.pan_id = 0xDECA;
  tx_packet.src = 0x0001;
  tx_packet.dst = 0xffff;
}

#define ALL_MSG_SN_IDX 2
#define ALL_MSG_COMMON_LEN 10
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
uint8_t frame_seq_nb = 0;
uint64_t poll_tx_ts, resp_rx_ts;
uint32_t final_tx_ts;
uint32_t final_tx_time;
int final_tx = 0;
void rx_err_cb(const dwt_cb_data_t *cb_data){
  printf("ERRR\n");
  //instance_info.diagnostics.uwb.rx.err_cb_count++;
}

void rx_to_cb(const dwt_cb_data_t *cb_data){
  printf("TO\n");
  //instance_info.diagnostics.uwb.rx.err_cb_count++;
}

void tx_conf_cb(const dwt_cb_data_t *cb_data){
  dwt_readtxtimestamp(&poll_tx_ts);
}

packet_std_t rx_packet;
uint64_t rx_ts;
int rx_num = 0;
void rx_ok_cb(const dwt_cb_data_t *cb_data){
  
  //printf("RX");
  dwt_readrxtimestamp(&rx_ts);
  rx_num ++;
  dwt_readrxdata(&rx_packet, cb_data->datalength, 0);
  //dwt_rxenable(DWT_START_RX_IMMEDIATE);
  switch(rx_packet.msg_type){
    case TWR_RESP:
      ranging_data = (packet_ranging_t *) tx_packet.payload;
      resp_rx_ts = rx_ts;
      ranging_data->resp_rx_ts[rx_packet.src] = resp_rx_ts;
      instance_info.node.rx_enable = 1;
  
  }

  //gpio_set(LED_TX);
  
  
}


void enable_intrupt_DW(){
  dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, NULL, &rx_err_cb, NULL, NULL);

  /* Enable wanted interrupts (TX confirmation, RX good frames,
   * RX timeouts and RX errors). */
  dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |
                   SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                   0,
                   DWT_ENABLE_INT);

  /* Clearing the SPI ready interrupt */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);

  /* Install DW IC IRQ handler. */
  port_set_dwic_isr(dwt_isr);
  dwt_setrxaftertxdelay(0);
}




uint64_t start_time = 0;
void instance_init(){
  init_NRF();
  uart_init();
  //port_set_dwic_isr(ali_isr);
  port_set_dw_ic_spi_fastrate();

  /* Reset DW IC */
  reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

  Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
  { };
  
  dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
      while (1)
      { };
  }
  
  instance_config_identity_init();
  enable_intrupt_DW();
  create_tx_packet();
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);
  init_LEDs();
  nrf_drv_uart_rx(&app_uart_inst, &UART_byte, 1);
  instance_info.node.tx_timestape = m_systick_cnt + instance_info.node.tx_delay_ms * 1000;
}
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    uint8_t i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8_t)ts;
        ts >>= 8;
    }
}

uint32_t status_reg;
#define PRE_TIMEOUT 5

//void instance_loop(){
//  printf("STart\n");
//  switch(instance_info.node.tx_msg_type){
//  case TWR_POLL:
//    dwt_writetxdata(PACKET_STD_HDR_LEN + FCS_LEN, &tx_packet, 0);
//    dwt_writetxfctrl(PACKET_STD_HDR_LEN + FCS_LEN, 0, 1);
//    int res = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
//    instance_info.node.tx_msg_type = TWR_FINAL;
//  break;
//    case TWR_FINAL:
//    tx_packet.msg_type = TWR_POLL;
//    break;
// }
//  Sleep(500);
//}


void instance_loop(){
  uint64_t t = m_systick_cnt;
  if (instance_info.node.tx_enable && t > instance_info.node.tx_timestape){
    instance_info.node.tx_enable = 0;
    //dwt_forcetrxoff();
    switch(instance_info.node.tx_msg_type){
      case TWR_POLL:
        
        printf("START...\n");  
        tx_packet.msg_type = TWR_POLL;
        dwt_writetxdata(PACKET_STD_HDR_LEN + FCS_LEN, &tx_packet, 0);
        dwt_writetxfctrl(PACKET_STD_HDR_LEN + FCS_LEN, 0, 1);
        int res = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
        instance_info.node.tx_msg_type = TWR_FINAL;
        instance_info.node.tx_enable = 1;
        gpio_set(LED_TX);instance_info.node.tx_timestape = m_systick_cnt + instance_info.twr.initiator.tx_final_dly_ms * 1000;
        //printf("%d\n", res);     
      break;
      case TWR_FINAL:
        gpio_reset(LED_TX);
        tx_packet.msg_type = TWR_FINAL;
        ranging_data = (packet_ranging_t *) (tx_packet.payload);
        dwt_readtxtimestamp(&poll_tx_ts);
        uint32_t current_ts = dwt_readsystimestamphi32();
        final_tx_time = (current_ts) + ((5000 * UUS_TO_DWT_TIME) >> 8);
        dwt_setdelayedtrxtime(final_tx_time);
        final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        ranging_data->poll_tx_ts = poll_tx_ts;
        ranging_data->final_tx_ts = final_tx_ts;
        dwt_forcetrxoff();
        dwt_writetxdata(PACKET_STD_HDR_LEN + sizeof(packet_ranging_t) + FCS_LEN, &tx_packet, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(PACKET_STD_HDR_LEN + sizeof(packet_ranging_t) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging bit set. */
        int ret = dwt_starttx(DWT_START_TX_DELAYED);
        ////printf("diff: %d\n", (current_ts - ranging_data->resp_rx_ts));
        //printf("%d\n", ret);
        memset(ranging_data, 0, sizeof(packet_ranging_t)); 
        instance_info.node.tx_timestape = m_systick_cnt + instance_info.node.tx_delay_ms * 1000;
        instance_info.node.tx_msg_type = TWR_POLL;
        instance_info.node.tx_enable = 1;
        rx_num = 0;
      break;
    
    };
    
  }
  if(instance_info.node.rx_enable){
    instance_info.node.rx_enable = 0;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }

}
