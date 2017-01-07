#include "stm32f3xx_hal.h"
#include "nrf24l01_defines.h"


extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


void low_csn(){
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void high_csn(){
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void low_ce(){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

void high_ce(){
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

void flush_tx_fifo(){
  uint8_t command;
  command = FLUSH_TX;
  
  low_csn();
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  high_csn();
}

void flush_rx_fifo(){
  uint8_t command;
  command = FLUSH_RX;
  
  low_csn();
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  high_csn();
}

void write_register(uint8_t reg, uint8_t value){
  uint8_t command, status_reg, written_value;
  uint8_t str[5];
  
  command = (W_REGISTER | (REGISTER_MASK & reg));
  
  low_csn();
  
  HAL_SPI_TransmitReceive(&hspi1, &command, &status_reg, 1, 100);
  //HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  high_csn();
  
  /*memset(str, 0, 5);
  sprintf(str, "%x", status_reg);
  HAL_UART_Transmit(&huart2, "\n\rSTATUS_REG = 0x", 17, 100);
  HAL_UART_Transmit(&huart2, str, 5, 100);*/
   
}

uint8_t read_register(uint8_t reg){
  uint8_t command, value, aux, status_reg;
  
  command = (R_REGISTER | (REGISTER_MASK & reg));
  aux = NOP;
  
  low_csn();
  
  HAL_SPI_TransmitReceive(&hspi1, &command, &status_reg, 1, 100);
  //HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  HAL_SPI_TransmitReceive(&hspi1, &aux, &value, 1, 100);
  
  high_csn();
  
  return value;
}

void write_bit(uint8_t reg, uint8_t pos, uint8_t value){
  uint8_t command, status_reg, old_value, new_value;
  uint8_t str[5];
  
  old_value = read_register(reg);
  command = (W_REGISTER | (REGISTER_MASK & reg));
  
  if(value){
    new_value = (old_value | (value << pos ));
  }else{
    new_value = old_value & (~(1 << pos));
  }
  
  
  write_register(reg, new_value);
  
}

uint8_t read_bit(uint8_t reg, uint8_t pos) {
	uint8_t tmp, aux;
	tmp = read_register(reg);
        
        aux = (tmp & (1 << pos));
        
	if (!aux) {
		return 0;
	}
	return 1;
}

void set_rx_mode(){
  write_bit(CONFIG, PRIM_RX, 1);
  high_ce();
  
  HAL_UART_Transmit(&huart2, "\n\r[f3]: RX MODE SET", 19, 100);
  
  /*uint8_t bufS[4];
  uint8_t config_reg;
  //lettura config (debug)
      config_reg = read_register(CONFIG);
      memset(bufS, 0, 4);
      sprintf(bufS, "%x", config_reg);
      HAL_UART_Transmit(&huart2, "\n\rCONFIG_REG = 0x", 17, 100);
      HAL_UART_Transmit(&huart2, bufS, 4, 100);*/
}

void set_tx_mode(){
  low_ce();
  write_bit(CONFIG, PRIM_RX, 0);
  
  HAL_UART_Transmit(&huart2, "\n\r[f3]: TX MODE SET", 19, 100);
  
  /*uint8_t bufS[4];
  uint8_t config_reg;
  //lettura config (debug)
      config_reg = read_register(CONFIG);
      memset(bufS, 0, 4);
      sprintf(bufS, "%x", config_reg);
      HAL_UART_Transmit(&huart2, "\n\rCONFIG_REG = 0x", 17, 100);
      HAL_UART_Transmit(&huart2, bufS, 4, 100);*/
}

void initialize_nrf24l01(){
  
  uint8_t command, value, i, status_reg;
  uint8_t value_v[5];
  
  HAL_UART_Transmit(&huart2, "\n\r[f3]: Initializing nRF24l01+..", 32, 1000);
  
  high_csn();
  low_ce(); //disabilito ricezione
  
  write_register(EN_AA, 0x03); //fase 2 0x01
  
  write_register(EN_RXADDR, 0x03); //0x01
  
  write_register(RF_CH, 0x50);
  
  write_register(RF_SETUP, 0x06);
  
  write_register(SETUP_AW, 0x03);
  
  write_register(RX_PW_P0, 0x06);
  
  //setto il registro RX ADDR P0 per la ricezione dell'ack
  
  low_csn();
  
  command = (W_REGISTER | (REGISTER_MASK & RX_ADDR_P0));
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  
  value = 0xAA;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x01;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  high_csn();
  
  //setto il registro RX ADDR P1 per la ricezione della frame nel caso GET e VER
  
  low_csn();
  
  command = (W_REGISTER | (REGISTER_MASK & RX_ADDR_P1));
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  
  value = 0xAA;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  high_csn();
  
  //setto l'indirizzo per la trasmissione
  
  low_csn();
  
  command = (W_REGISTER | (REGISTER_MASK & TX_ADDR));
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
   
  value = 0xAA;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  value = 0x01;
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  high_csn();
  
  write_register(DYNPD, 0x03);  //0x01
  
  write_register(FEATURE, 0x07);
  
  write_register(CONFIG, 0x0E); //3F
  
  flush_tx_fifo();
  flush_rx_fifo();
  
  uint8_t aux, aus;
  uint8_t str[5];
  aux = NOP;
  
  /*low_csn();
  command = (R_REGISTER | (REGISTER_MASK & CONFIG));
  HAL_SPI_TransmitReceive(&hspi1, &command, &status_reg, 1, 100);
  //HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  HAL_SPI_TransmitReceive(&hspi1, &aux, &aus, 1, 100);
  high_csn();
  
  memset(str, 0, 5);
  sprintf(str, "%x", aus);
  HAL_UART_Transmit(&huart2, "\n\rCONFIG_REG = 0x", 17, 100);
  HAL_UART_Transmit(&huart2, str, 5, 100);
  */
  
}

void send_payload(uint8_t *b){
  
  uint8_t command, i, status_reg;
  uint8_t str[5];
  
  flush_tx_fifo();
  
  /*write_bit(STATUS, RX_DR, 1);
  write_bit(STATUS, TX_DS, 1);
  write_bit(STATUS, MAX_RT, 1);*/
  
  low_ce();
  low_csn();
  
  command = W_TX_PAYLOAD;
  //HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  HAL_SPI_TransmitReceive(&hspi1, &command, &status_reg, 1, 100);
  
  
  //HAL_SPI_Transmit(&hspi1, b, PAYLOAD_LENGHT, 100);
  for(i = 0; i < PAYLOAD_LENGTH; i++){
    HAL_SPI_Transmit(&hspi1, &b[i], 1, 100);
  }
  
  high_csn();
  
  high_ce();
  /*HAL_Delay(1);
  low_ce();*/
  
  /*memset(str, 0, 5);
  sprintf(str, "%x", status_reg);
  HAL_UART_Transmit(&huart2, "\n\rSTATUS_REG = 0x", 17, 100);
  HAL_UART_Transmit(&huart2, str, 5, 100);*/
  
}

void read_payload(){
  uint8_t r_payload[PAYLOAD_LENGTH];
  uint8_t command, i, status_reg, aux, value;
  uint8_t str[100];
  
  memset(str, 0, 100);
  aux = NOP;
  
  low_csn();
  
  command = R_RX_PAYLOAD;
  HAL_SPI_TransmitReceive(&hspi1, &command, &status_reg, 1, 100);
  
  for(i = 0; i < PAYLOAD_LENGTH; i++){
    HAL_SPI_TransmitReceive(&hspi1, &aux, &value, 1, 100);
    r_payload[i] = value;
  }
  
  high_csn();
  
  sprintf(str, "\n\rRESPONSE:\n\r\tcommand: %x \n\r\tid_var: %x \n\r\tvalue: %x%x%x%x\n\n", r_payload[0], r_payload[1], r_payload[2], r_payload[3], r_payload[4], r_payload[5]);
  //HAL_UART_Transmit(&huart2, "\n\rSTATUS_REG = 0x", 17, 100);
  HAL_UART_Transmit(&huart2, str, 100, 1000);
  
}