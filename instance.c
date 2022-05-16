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
#include "mac_802_15_4.h"
#include "nrf_drv_uart.h"
static void uart_event_handler(nrf_drv_uart_event_t * p_event, void* p_context)
{
    if (p_event->type == NRF_DRV_UART_EVT_RX_DONE)
    {
        if (p_event->data.rxtx.bytes)
        {
            // Event to notify that data has been received
        }
    }
    else if (p_event->type == NRF_DRV_UART_EVT_ERROR)
    {
        // Event to notify that an error has occured in the UART peripheral
    }
    else if (p_event->type == NRF_DRV_UART_EVT_TX_DONE)
    {
       // Event to notify that the last byte from FIFO has been transmitted

    }
}
static nrf_drv_uart_t app_uart_inst = NRF_DRV_UART_INSTANCE(APP_UART_DRIVER_INSTANCE);
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


uint8_t UART_msg_payload[200];
uint32_t sequence_numbers[10000];
uint8_t rx_buffer[100];
mac_frame_802_15_4_format_t mac_frame;

uint32_t seq_num = 0;
uint32_t tx_num = 0;
packet_std_t tx_packet;
packet_std_t rx_packet;
packet_info_t payload;
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

uint8_t tx_msg[] = {0x61, 0x88, 0, 0x0, 0x0, 0x1, 0x0, 'X', 'T', 'm', 'a', 'c', 'p', 'a', 'y', 'l', 'o', 'a', 'd'};
void init_tx_packet(){
  tx_packet.src = identity_get_address();
  if(identity_get_operations() & IDENTITY_OPERATIONS_TIMESYNC){
    tx_packet.packet_id = (MSG_TIME_SYNC);
    tx_packet.sequence_number = (0X00000000);
  }
  if(identity_get_operations() & (IDENTITY_OPERATIONS_DATA_TX | IDENTITY_OPERATIONS_CONSTANT_TX)){
    tx_packet.packet_id = (MSG_DATA);
    tx_packet.packet_id = 0x8861;
    //tx_packet.packet_id = 0x1010;
    tx_packet.sequence_number = (0X00000000);
  }
  tx_packet.src = identity_get_address();
  tx_packet.dst = instance_info.config.dst_addr;
  payload.sequence_number = 0;
  memcpy(tx_packet.payload, (uint8_t *)&payload, sizeof(payload));
  if (dwt_writetxdata(sizeof(instance_info.config.packet_size), &tx_packet, 0) != DWT_SUCCESS){
    
  }
  dwt_writetxfctrl(sizeof(instance_info.config.packet_size), 0, 0);
  

}

void gpio_set(uint16_t port){
	dwt_or16bitoffsetreg(GPIO_OUT_ID, 0, (port));
}

void gpio_reset(uint16_t port){
	dwt_and16bitoffsetreg(GPIO_OUT_ID, 0, (uint16_t)(~(port)));
}

static uint32_t m_systick_cnt;
int flg = 0;


void SysTick_Handler(void) {
    m_systick_cnt++;
}

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






uint8_t set_radio_params(){

  if (instance_info.config.cca_wait != 0){
    dwt_setpreambledetecttimeout(instance_info.config.cca_wait);
    return DWT_START_TX_CCA;
  }
  //return DWT_START_TX_IMMEDIATE;
}




void instance_tx(void){
  uint32_t n1, n2;
  uint8_t mode;
  mode = set_radio_params();
  tx_packet.sequence_number = instance_info.config.sequence_number;
  packet_info_t *payload = tx_packet.payload;
  payload->sequence_number = instance_info.config.sequence_number;
  dwt_writetxdata(50, (uint8_t *) &tx_packet, 0);
  dwt_writetxfctrl(instance_info.config.packet_size, 0, 0);
  //n1 = m_systick_cnt;
  //while(m_systick_cnt - n1 < 1000){
  //  nrf_delay_us(1);
  //}
  if (dwt_starttx(mode) != DWT_SUCCESS ){
    instance_info.diagnostics.uwb.tx.failed_count++;
  }
  gpio_set(LED_TX);


}

void instance_rx(void){
  uint8_t mode;
  dwt_setpreambledetecttimeout(0);
  if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS ){
          instance_info.diagnostics.uwb.rx.enable_error++;
  }
}

void process_message(void){
  if(identity_get_operations() & IDENTITY_OPERATIONS_DATA_TX){
    if (rx_packet.src == 0){
      tx_num = 0;
      instance_tx();
    }
  }
  if(identity_get_operations() & IDENTITY_OPERATIONS_DATA_RX){
    instance_rx();
  }

}


void sfd_ok_cb(const dwt_cb_data_t *cb_data){
  //process_message();
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void rx_to_cb(const dwt_cb_data_t *cb_data){
  //gpio_set(LED_RX);
	//instance_info.diagnostics.uwb.rx.to_cb_count++;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void rx_err_cb(const dwt_cb_data_t *cb_data){
  instance_info.diagnostics.uwb.rx.err_cb_count++;
}


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

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
      while (1)
      { };
  }
  instance_config_identity_init();
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);
  init_LEDs();
  ring_buffer_init();
  if (identity_get_operations() & (IDENTITY_OPERATIONS_DATA_TX | IDENTITY_OPERATIONS_CONSTANT_TX | IDENTITY_OPERATIONS_TIMESYNC)){
    init_tx_packet();  
  }
  //dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);

  if(identity_get_operations() & (IDENTITY_OPERATIONS_DATA_RX)){
    //dwt_setaddress16(0x0003);
    //dwt_setpanid(1);
    instance_rx();
  }
  if(identity_get_operations() & IDENTITY_OPERATIONS_CONSTANT_TX)
    instance_tx();
  if(identity_get_operations() & IDENTITY_OPERATIONS_DATA_TX)
    instance_rx();
}

void send_UART_msg(uint8_t *msg, uint8_t payload_len){
  uint8_t pointer = 0;
  if(payload_len > 200){
    return -1;
  }
  UART_msg_payload[0] = 0x5C;
  UART_msg_payload[1] = 0x51;
  UART_msg_payload[2] = payload_len;
  memcpy(UART_msg_payload + 3, msg, payload_len);
  uint32_t ret =  send_uart(UART_msg_payload, payload_len + 3);
  //printf("alireza\n");
  return;
}

int seq_indicator = 10;
int noise_preamble_indicator = 0;
Radio_action rx_handle_cb(){
  dwt_readrxdata( &rx_packet, 30, 0);
  if (1){
    gpio_set(PORT_DE);
    gpio_reset(PORT_DE);
    packet_info_t *payload = rx_packet.payload;
    printf("%d\n",payload->sequence_number);
    
  }
  if (identity_get_operations() & IDENTITY_OPERATIONS_DATA_TX)
    return ACTION_TX;
  if (identity_get_operations() & IDENTITY_OPERATIONS_DATA_RX)
    return ACTION_RX;
}

Radio_action tx_handle_cb(){
    instance_info.config.sequence_number++;
     if (
       instance_info.config.tx_number != 0 && 
       instance_info.config.sequence_number >= instance_info.config.tx_number
     ){
      gpio_set(LED_RX);
      gpio_set(LED_TX);
      return ACTION_NONE;
    }
    if (identity_get_operations() & IDENTITY_OPERATIONS_DATA_TX){
      return ACTION_RX;   
    }
    if (identity_get_operations() & IDENTITY_OPERATIONS_CONSTANT_TX)
      return ACTION_TX;
}


Radio_action rx_err_handle_cb(){
  noise_preamble_indicator++;
  return ACTION_RX;
}

Radio_action cca_fail_handle_cb(){
  return ACTION_TX;
}

uint32_t status_reg, status_regh;
uint32_t ts1, ts2;
int err_counter = 0;
void instance_loop(){
  Radio_action act = ACTION_NONE;
  int cca_flg = 0;
  status_reg = dwt_read32bitreg(SYS_STATUS_ID);
  status_regh = dwt_read32bitreg(SYS_STATUS_HI_ID);
  if (status_reg & SYS_ENABLE_LO_TXFRB_ENABLE_BIT_MASK){
    gpio_reset(LED_TX);
    gpio_set(LED_TX);
  }

  if (status_reg & SYS_ENABLE_LO_TXPRS_ENABLE_BIT_MASK){
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRB_BIT_MASK);
    gpio_set(LED_TX);
  }

  if ((status_reg & SYS_STATUS_TXFRS_BIT_MASK)){
    gpio_reset(LED_TX);
    dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, (uint8_t)SYS_STATUS_ALL_TX);
    gpio_reset(PORT_DE);
    cca_flg = 0;
    act = tx_handle_cb();
  }


  if((status_regh & SYS_STATUS_HI_CCA_FAIL_BIT_MASK) && ((status_reg & SYS_STATUS_TXFRS_BIT_MASK) == 0)){
    dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, (uint8_t)SYS_STATUS_ALL_TX);
    dwt_forcetrxoff();
    gpio_set(PORT_DE);
    gpio_reset(LED_TX);
    cca_flg = 1;
    act = cca_fail_handle_cb();
  }


  if (status_reg & SYS_ENABLE_LO_RXPRD_ENABLE_BIT_MASK){
     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPRD_BIT_MASK);
     ts1 = m_systick_cnt; 
     gpio_set(LED_RX);
  }
  if (status_reg & SYS_ENABLE_LO_RXPHD_ENABLE_BIT_MASK){
    dwt_write32bitreg(SYS_STATUS_ID, SYS_ENABLE_LO_RXSFDD_ENABLE_BIT_MASK);
    //gpio_reset(LED_RX);
    //gpio_set(LED_RX);

  }
  if (status_reg & SYS_STATUS_ALL_RX_ERR){
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    ts2 = m_systick_cnt;
    printf("%d, %d,", err_counter, ts2-ts1);
    err_counter++;
    if(status_reg & SYS_STATUS_RXPHE_BIT_MASK)
      printf("PHE\n");
    if(status_reg & SYS_STATUS_RXFCE_BIT_MASK)
      printf("FCE\n");
    if(status_reg & SYS_STATUS_RXFSL_BIT_MASK)
      printf("FSL\n");
    if(status_reg & SYS_STATUS_RXSTO_BIT_MASK)
      printf("STO\n");
    if(status_reg & SYS_STATUS_CIAERR_BIT_MASK)
      printf("CIA\n");
    if(status_reg & SYS_STATUS_ARFE_BIT_MASK)
      printf("aref\n");

    gpio_reset(LED_RX); 
    act = rx_err_handle_cb();
    
  }
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK){
    gpio_reset(LED_RX);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    ts2 = m_systick_cnt;
    printf("%d,", ts2-ts1);
    act = rx_handle_cb();
  }
 
  if (act == ACTION_RX){
    instance_rx();
  }
  if (act == ACTION_TX){
    if(!cca_flg){
      Sleep(instance_info.config.IPI_wait);
    }
    instance_tx();
  }

}
