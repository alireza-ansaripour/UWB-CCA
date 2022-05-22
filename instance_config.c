/*
 * instance_config.c
 *
 *  Created on: Oct 14, 2019
 *      Author: milad
 */

#include "instance_config.h"
#include "identity.h"
#include "instance.h"
#include "port.h"
#define PAN_ID      0
//#define SHORT_ADDR  0x0200 /* "RX" */
#define SHORT_ADDR  0x0000 /* "RX" */
//#define SHORT_ADDR  0x003 /* "RX" */
instance_info_t instance_info;



dwt_config_t radio_config = {
    5,               /* Channel number. */
    DWT_PLEN_512,    /* Preamble length. Used in TX only. */
    DWT_PAC32,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,      /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (4096 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};
void identity_set_role(Role role);
dwt_txconfig_t txconfig =
{
    0x34,           /* PG delay. */
    0xfdfdfdfd,      /* TX power. */
    0x0             /*PG count*/
};


packet_std_t ack_pkt;


void instance_config_identity_init() {
  uint32_t dev_id = NRF_FICR->DEVICEADDR[0];
  init_config(); 
  switch(dev_id){
    case 4113761924:
      identity_set_address(0x0001);
      identity_set_role(ROLE_TS);
      instance_info.config.tx_number = 500;
      //radio_config.txCode = 10;
    break;
    case 153137759:
      identity_set_address(0x0002);
      identity_set_role(ROLE_TX_TS);
      instance_info.config.tx_after_rx_wait = 1500;
      radio_config.txCode = 10;
    break;
    case 1697349500:
      identity_set_address(0x0003);
      identity_set_role(ROLE_TX);
      instance_info.config.IPI_wait = 1000;
      radio_config.txCode = 11;
      instance_info.config.tx_number=0;
    break;
    case 1649967333:
      identity_set_role(ROLE_RX);
      radio_config.rxCode = 11;
    break;
      

  }
  configure_node();
  dwt_setpanid(PAN_ID);
  //dwt_seteui(eui);
  //dwt_setaddress16(SHORT_ADDR);
  //dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
}

void identity_set_role(Role role) {
  identity_set_operations(IDENTITY_OPERATIONS_NONE);
  switch (role) {
    case ROLE_TX:
      identity_append_operations(IDENTITY_OPERATIONS_CONSTANT_TX);
      
      break;
    case ROLE_RX:
      identity_append_operations(IDENTITY_OPERATIONS_DATA_RX);
    break;

    case ROLE_TS:
      identity_append_operations(IDENTITY_OPERATIONS_TIMESYNC);
      identity_append_operations(IDENTITY_OPERATIONS_CONSTANT_TX);
      instance_info.config.tx_number = 0;
    break;
    case ROLE_TX_TS:
      identity_append_operations(IDENTITY_OPERATIONS_DATA_TX);
      instance_info.config.tx_number = 0;
    break;
  }
}

void init_config() {
  instance_info.config.radio_config = radio_config;
  instance_info.config.packet_size = 100;
  if (identity_get_operations() & IDENTITY_OPERATIONS_CONSTANT_TX)
    instance_info.config.tx_number = TX_PKT_CNT;
  else
    instance_info.config.tx_number = 0;
  instance_info.config.tx_config = txconfig;
  instance_info.config.sequence_number = 0;
  instance_info.config.IPI_wait = 100;
  //instance_info.config.tx_after_rx_wait = DELAY_TX;
  instance_info.events.tx_enable = 0;
}

void configure_node() {
  if(dwt_configure(&radio_config)) {
    while (1)
    { };
  }
  dwt_configuretxrf(&txconfig);
}

