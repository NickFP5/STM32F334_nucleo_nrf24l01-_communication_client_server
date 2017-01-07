#include "stm32f3xx_hal.h"


#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17


/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

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

void initialize_nrf24l01(){
  
  uint8_t command;
  
  low_csn();
  
  command = (EN_AA | (REGISTER_MASK & 0x01));
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  
  command = (EN_RXADDR | (REGISTER_MASK & 0x01));
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  
  command = (RF_CH | (REGISTER_MASK & 0x50));
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  
  command = (RF_SETUP | (REGISTER_MASK & 0x06));
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  
  command = (SETUP_AW | (REGISTER_MASK & 0x03));
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  
  command = (CONFIG | (REGISTER_MASK & 0x3F));
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  
  high_csn();
  
  low_ce(); //disabilito ricezione
  
}

void send_bytes(uint8_t *b){
  
  uint8_t command, i, status_reg;
  
  low_ce();
  low_csn();
  
  command = W_TX_PAYLOAD;
  //HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  HAL_SPI_TransmitReceive(&hspi1, &command, &status_reg, 1, 100);
  
  HAL_UART_Transmit(&huart2, &status_reg, 1, 100);
  
  for(i = 0; i < PAYLOAD_LENGHT; i++){
    HAL_SPI_Transmit(&hspi1, &b[i], 1, 100);
    high_ce();
    low_ce();
  }
  
  high_csn();
  
}