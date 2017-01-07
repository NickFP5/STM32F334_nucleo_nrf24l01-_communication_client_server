/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "pagano_library.h"
#define PAYLOAD_LENGHT  6
#define BUFRXDIM 360
    
    
    
    
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
#define DYNPD       0x1C
#define FEATURE     0x1D


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
    

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0        
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
    
    
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void initialize_nrf24l01();
void send_bytes(uint8_t *b);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t bufferURx; //singolo byte ricevuto dietro IT da UART1
uint8_t bufferSRx; //singolo byte ricevuto da SPI
uint8_t bufferSTx[PAYLOAD_LENGHT]; //buffer di 6 byte trasmesso su SPI
uint8_t stateU, j, a, k; //stateU e' un flag che indica che è possibile stampare il buffer buf
uint8_t buf[BUFRXDIM]; //accumulatore di caratteri per stampa complessiva
uint8_t bufS[4]; //stringa usata per stampare il valore esadecimale inviato tramite SPI

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef stato; //per il confrollo del risultato delle operazioni di transmit o receive
  int c, i;
  stateU =  0;
  j = a = k = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &bufferURx, 1);
  initialize_nrf24l01();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
    if(stateU == 1){ //flag per indicare che è possibile stampare il buffer buf
      
      if(j == 0)HAL_UART_Transmit(&huart2, buf, BUFRXDIM, 1000);
      else {
        for(c = 0; c < j; c++)HAL_UART_Transmit(&huart2, &buf[c], 1, 100); //stampo a schermo il buffer buf a schermo fino all'ultimo carattere significativo
      }
      stateU = 0;
      j = 0;

    }
    
    if(a % 10 == 0){ //comportamento schedulato ogni 3 secondi (per leggibilita' su docklight)
    
      /*stato = HAL_SPI_Receive(&hspi1, &bufferSRx, 1, 300);
      if(stato == HAL_OK){
        HAL_UART_Transmit(&huart2, "\n\rSPI[f0]: ", 11, 100);
        HAL_UART_Transmit(&huart2, &bufferSRx, 1, 100);
        HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
      }*/
      
      /*memset(&hspi1.Instance->DR, 0, 16);
      
      HAL_SPI_TransmitReceive(&hspi1, &bufferSTx, &bufferSRx, 1, 100);*/
    
      for(i = 0; i < PAYLOAD_LENGHT; i++){
        bufferSTx[i] = k + i; //preparo un nuovo valore da trasmettere
      }
      //sprintf(bufS, "%x", bufferSTx); //salvo tale valore su bufS per poterlo stampare su docklight
      
      //stato = HAL_SPI_Transmit(&hspi1, &bufferSTx, 1, 100);
      //stato = HAL_SPI_TransmitReceive(&hspi1, &bufferSTx, &bufferSRx, 1, 100); //trametto e ricevo via SPI
      send_bytes(bufferSTx);
      HAL_UART_Transmit(&huart2, "\n\rSPI[f3]: 0x", 13, 100);
      for(i = 0; i < PAYLOAD_LENGHT; i++){
        sprintf(bufS, "%x", bufferSTx[i]); //salvo tale valore su bufS per poterlo stampare su docklight
        HAL_UART_Transmit(&huart2, bufS, 4, 100); //stampo il valore trasmesso tramite SPI
        HAL_UART_Transmit(&huart2, " ", 1, 100);
      }

      HAL_UART_Transmit(&huart2, "\n\r", 2, 100);
        
      
      a = 0;
      k++;
    }
    
    HAL_Delay(300);
    a++;
    

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  HAL_SPI_Init(&hspi1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CE_Pin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin PA8 */
  GPIO_InitStruct.Pin = CSN_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSN_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

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

void write_register(uint8_t reg, uint8_t value){
  uint8_t command, status_reg, written_value;
  uint8_t str[5];
  
  command = (W_REGISTER | (REGISTER_MASK & reg));
  
  low_csn();
  
  HAL_SPI_TransmitReceive(&hspi1, &command, &status_reg, 1, 100);
  //HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  
  high_csn();
  
  memset(str, 0, 5);
  sprintf(str, "%x", status_reg);
  HAL_UART_Transmit(&huart2, "\n\rSTATUS_REG = 0x", 17, 100);
  HAL_UART_Transmit(&huart2, str, 5, 100);
   
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
  new_value = (old_value | (value << pos ));
  
  
  write_register(reg, new_value);
  
  
}

void initialize_nrf24l01(){
  
  uint8_t command, value, i, status_reg;
  uint8_t value_v[5];
  
  high_csn();
  low_ce(); //disabilito ricezione
  
  write_register(EN_AA, 0x01);
  
  write_register(EN_RXADDR, 0x01);
  
  write_register(RF_CH, 0x50);
  
  write_register(RF_SETUP, 0x06);
  
  write_register(SETUP_AW, 0x03);
  
  write_register(RX_PW_P0, 0x06);
  
  //setto il registro RX ADDR P0 per la ricezione dell'ack
  
  low_csn();
  
  command = (W_REGISTER | (REGISTER_MASK & RX_ADDR_P0));
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  for(i = 0; i < 5; i++){
    HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  }
  
  high_csn();
  
  //setto l'indirizzo per la trasmissione
  
  low_csn();
  
  command = (W_REGISTER | (REGISTER_MASK & TX_ADDR));
  value = 0x00;
  HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  for(i = 0; i < 5; i++){
    HAL_SPI_Transmit(&hspi1, &value, 1, 100);
  }
  
  high_csn();
  
  write_register(DYNPD, 0x01);
  
  write_register(FEATURE, 0x07);
  
  write_register(CONFIG, 0x3E); //3F
  
  
  
  uint8_t aux, aus;
  uint8_t str[5];
  aux = NOP;
  
  low_csn();
  command = (R_REGISTER | (REGISTER_MASK & CONFIG));
  HAL_SPI_TransmitReceive(&hspi1, &command, &status_reg, 1, 100);
  //HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  HAL_SPI_TransmitReceive(&hspi1, &aux, &aus, 1, 100);
  high_csn();
  
  memset(str, 0, 5);
  sprintf(str, "%x", aus);
  HAL_UART_Transmit(&huart2, "\n\rCONFIG_REG = 0x", 17, 100);
  HAL_UART_Transmit(&huart2, str, 5, 100);
  
  
}

void send_bytes(uint8_t *b){
  
  uint8_t command, i, status_reg;
  uint8_t str[5];
  
  flush_tx_fifo();
  
  write_bit(STATUS, RX_DR, 1);
  write_bit(STATUS, TX_DS, 1);
  write_bit(STATUS, MAX_RT, 1);
  
  low_ce();
  low_csn();
  
  command = W_TX_PAYLOAD;
  //HAL_SPI_Transmit(&hspi1, &command, 1, 100);
  HAL_SPI_TransmitReceive(&hspi1, &command, &status_reg, 1, 100);
  
  
  //HAL_SPI_Transmit(&hspi1, b, PAYLOAD_LENGHT, 100);
  for(i = 0; i < PAYLOAD_LENGHT; i++){
    HAL_SPI_Transmit(&hspi1, &b[i], 1, 100);
  }
  
  high_csn();
  
  high_ce();
  HAL_Delay(1);
  low_ce();
  
  memset(str, 0, 5);
  sprintf(str, "%x", status_reg);
  HAL_UART_Transmit(&huart2, "\n\rSTATUS_REG = 0x", 17, 100);
  HAL_UART_Transmit(&huart2, str, 5, 100);
  
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  char aux = (char)bufferURx; //memorizzo il carattere in una variabile di appoggio
  
  if(huart == &huart1){
    
    buf[j] = bufferURx;
    j = (j+1)%BUFRXDIM;
    
    if(j == 0 || aux == '\r'){//è stata raggiunta la dimensione massima del buffer buf o è stato incontrato un ritorno a capo
      
        stateU = 1; //flag per indicare che il buffer buf può essere letto nel main
        
    }
    
    HAL_UART_Receive_IT(&huart1, &bufferURx, 1); //riabilito l'interrupt
    return;
    
  }
  
  
}


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
