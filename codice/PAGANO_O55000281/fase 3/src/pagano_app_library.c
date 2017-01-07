#include "stm32f3xx_hal.h"
#include "nrf24l01_defines.h"
#include "pagano_dll_library.h"


extern UART_HandleTypeDef huart2;


void GET_command(uint8_t id_var){
  uint8_t buffer[6];
  uint8_t str[20];

  buffer[0] = 0x01;
  buffer[1] = id_var;

  buffer[2] = NOP;
  buffer[3] = NOP;
  buffer[4] = NOP;
  buffer[5] = NOP;
  
  set_tx_mode();
  
  send_payload(buffer);

  memset(str, 0, 20);
  sprintf(str, "\n\r[f3]: GET 0x%x 0x%x", buffer[0], buffer[1]);
  HAL_UART_Transmit(&huart2, str, 20, 200);

}

void SET_command(uint8_t id_var, uint8_t * value){
  uint8_t buffer[6];
  uint8_t i, j;
  uint8_t str[50];
  
  buffer[0] = 0x03;
  buffer[1] = id_var;
  
  j = 0;
  for(i = 2; i < PAYLOAD_LENGTH; i++){
    buffer[i] = value[j];
    j++;
  }
  
  set_tx_mode();
  
  send_payload(buffer);

  memset(str, 0, 50);
  sprintf(str, "\n\r[f3]: SET 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
  HAL_UART_Transmit(&huart2, str, 50, 500);
  
}

void VER_command(){
  uint8_t buffer[6];
  uint8_t str[20];
  
  buffer[0] = 0x04;
  buffer[1] = NOP;
  buffer[2] = NOP;
  buffer[3] = NOP;
  buffer[4] = NOP;
  buffer[5] = NOP;
  
  set_tx_mode();
  
  send_payload(buffer);
  
  memset(str, 0, 20);
  sprintf(str, "\n\r[f3]: VER 0x%x", buffer[0]);
  HAL_UART_Transmit(&huart2, str, 20, 200);
  
}