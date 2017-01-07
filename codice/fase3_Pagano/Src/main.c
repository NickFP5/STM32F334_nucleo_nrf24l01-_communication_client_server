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
#include "nrf24l01_defines.h"
#include "pagano_dll_library.h"
#include "pagano_app_library.h"
    
#define PAYLOAD_LENGTH  6
#define BUFRXDIM 360
    

    
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
uint8_t bufferSTx[PAYLOAD_LENGTH]; //buffer di 6 byte trasmesso su SPI
uint8_t stateU, j, a, k; //stateU e' un flag che indica che è possibile stampare il buffer buf
uint8_t buf[BUFRXDIM]; //accumulatore di caratteri per stampa complessiva
uint8_t bufS[4]; //stringa usata per stampare il valore esadecimale inviato tramite SPI
uint8_t statoLed, config_reg;
uint8_t set_value[4];
uint8_t get_sent;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  //HAL_StatusTypeDef stato; //per il confrollo del risultato delle operazioni di transmit o receive
  int c;
  stateU =  0;
  statoLed = 0;
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
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
    if(stateU == 1){ //flag per indicare che è possibile stampare il buffer buf
      
      if(j == 0){
        HAL_UART_Transmit(&huart2, "\n\r", 2, 1000);
        HAL_UART_Transmit(&huart2, buf, BUFRXDIM, 1000);
      }
      else {
        HAL_UART_Transmit(&huart2, "\n\r", 2, 1000);
        for(c = 0; c < j; c++)HAL_UART_Transmit(&huart2, &buf[c], 1, 100); //stampo a schermo il buffer buf a schermo fino all'ultimo carattere significativo
      }
      stateU = 0;
      j = 0;

    }
    
    if(a % 2 == 0){ //comportamento schedulato ogni 1 sec
    
      if(k % 2 == 0){ //cambio lo stato dei led
        statoLed = (statoLed + 1) % 4;
        
        set_value[0] = statoLed;
        
        //set_tx_mode();
        SET_command(LED, set_value);
        
        
      }else{ //GET var id 1 o 2
        if(k % 3 == 1){ //var 2
          //set_tx_mode();
          GET_command(INC);
        }else{ //var 1
          //set_tx_mode();
          GET_command(TIM);
        }
       
        
      }
      
    
      /*for(i = 0; i < PAYLOAD_LENGHT; i++){
        bufferSTx[i] = k + i; //preparo un nuovo valore da trasmettere
      }
      
      send_payload(bufferSTx);
      HAL_UART_Transmit(&huart2, "\n\rSPI[f3]: 0x", 13, 100);
      for(i = 0; i < PAYLOAD_LENGHT; i++){
        sprintf(bufS, "%x", bufferSTx[i]); //salvo tale valore su bufS per poterlo stampare su docklight
        HAL_UART_Transmit(&huart2, bufS, 4, 100); //stampo il valore trasmesso tramite SPI
        HAL_UART_Transmit(&huart2, " ", 1, 100);
      }

      HAL_UART_Transmit(&huart2, "\n\r", 2, 100);*/
        
      
      
      
      
      a = 0;
      k = (k+1)%4;
    }
    
    /*//lettura config (debug)
      config_reg = read_register(CONFIG);
      memset(bufS, 0, 4);
      sprintf(bufS, "%x", config_reg);
      HAL_UART_Transmit(&huart2, "\n\rCONFIG_REG = 0x", 17, 100);
      HAL_UART_Transmit(&huart2, bufS, 4, 100);
      //lettura status (debug)
      config_reg = read_register(STATUS);
      memset(bufS, 0, 4);
      sprintf(bufS, "%x", config_reg);
      HAL_UART_Transmit(&huart2, "\n\rSTATUS_REG = 0x", 17, 100);
      HAL_UART_Transmit(&huart2, bufS, 4, 100);*/
      /*//lettura enAA (debug)
      config_reg = read_register(EN_AA);
      memset(bufS, 0, 4);
      sprintf(bufS, "%x", config_reg);
      HAL_UART_Transmit(&huart2, "\n\rEN_AA_REG = 0x", 16, 100);
      HAL_UART_Transmit(&huart2, bufS, 4, 100);
      //lettura enRXADDR (debug)
      config_reg = read_register(EN_AA);
      memset(bufS, 0, 4);
      sprintf(bufS, "%x", config_reg);
      HAL_UART_Transmit(&huart2, "\n\rEN_RXADDR_REG = 0x", 20, 100);
      HAL_UART_Transmit(&huart2, bufS, 4, 100);*/
    
    HAL_Delay(500);
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
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSN_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  
  uint8_t value;
  
  HAL_UART_Transmit(&huart2, "\n\r\tInterrupt on IRQ PIN", 23, 200);
  
  value = read_bit(STATUS, TX_DS);
  if(value == 1){ //ACK ricevuto
    
    HAL_UART_Transmit(&huart2, "\n\r\tTX_DS Interrupt\n\r", 20, 200);
    
    write_bit(STATUS, TX_DS, 1);
    set_rx_mode();
    
  }else{
    
    value = read_bit(STATUS, RX_DR);
    
    if(value == 1){ //ricezione completata
      
      HAL_UART_Transmit(&huart2, "\n\r\tRX_DR Interrupt\n\r", 20, 200);
      
      write_bit(STATUS, RX_DR, 1);
      read_payload();  
      
    }else{
      
      value = read_bit(STATUS, MAX_RT);
      if(value == 1){ // MAX RT asserted
        
        HAL_UART_Transmit(&huart2, "\n\r\tMAX_RT Interrupt\n\r", 21, 200);
        
        write_bit(STATUS, MAX_RT, 1);
        flush_tx_fifo();  
        
      }
    }
  }
  
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
