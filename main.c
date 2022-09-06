
#include "instance.h"
#include "deca_regs.h"
#include "deca_device_api.h"
//uint8_t rx_buffer_main[100];
int main(void){
  instance_init();
  while(1){
    instance_loop();
  }


}
