/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId taskMotorLigadoHandle;
osThreadId btnLigarMtrHandle;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
void StartTask03(void const * argument);
void ligarMotor(void const * argument);

/* USER CODE BEGIN PFP */
void trataInterrupcao(uint16_t);
int checkDebounce(uint16_t,GPIO_TypeDef  *);
static QueueHandle_t Queue_StartMotor = NULL;
static QueueHandle_t Queue_StopMotor  = NULL;
static QueueHandle_t Queue_delayLedAzul  = NULL;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  Queue_StartMotor = xQueueCreate( 10, sizeof(int*));
  Queue_StopMotor = xQueueCreate( 10, sizeof(int*));
  Queue_delayLedAzul = xQueueCreate( 5, sizeof(int*));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of taskMotorLigado */
  osThreadDef(taskMotorLigado, StartTask03, osPriorityIdle, 0, 128);
  taskMotorLigadoHandle = osThreadCreate(osThread(taskMotorLigado), NULL);

  /* definition and creation of btnLigarMtr */
  osThreadDef(btnLigarMtr, ligarMotor, osPriorityHigh, 0, 128);
  btnLigarMtrHandle = osThreadCreate(osThread(btnLigarMtr), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LedePCI_GPIO_Port, LedePCI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Motor_Pin|LedAzul_Pin|LedVerde_Pin|LedAmarelo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LedePCI_Pin */
  GPIO_InitStruct.Pin = LedePCI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LedePCI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Caixa_NivelAlto_Pin Poco_NivelBaixo_Pin ligarMotor1min_Pin ligarMotor5min_Pin
                           ligarMotor10min_Pin */
  GPIO_InitStruct.Pin = Caixa_NivelAlto_Pin|Poco_NivelBaixo_Pin|ligarMotor1min_Pin|ligarMotor5min_Pin
                          |ligarMotor10min_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Caixa_NivelBaixo_Pin Poco_NivelAlto_Pin */
  GPIO_InitStruct.Pin = Caixa_NivelBaixo_Pin|Poco_NivelAlto_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_Pin LedAzul_Pin LedVerde_Pin LedAmarelo_Pin */
  GPIO_InitStruct.Pin = Motor_Pin|LedAzul_Pin|LedVerde_Pin|LedAmarelo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
BaseType_t xHigherPriorityTaskWoken;
void trataInterrupcao(uint16_t GPIO_Pin){
  switch(GPIO_Pin){
    case Poco_NivelBaixo_Pin:{
      if(checkDebounce(Poco_NivelBaixo_Pin, GPIOA) == 0){
          uint8_t paradaTipo = 2;
          xQueueSendToFrontFromISR(Queue_StopMotor, &paradaTipo, &xHigherPriorityTaskWoken);
      }
    }
    break;
    case Caixa_NivelAlto_Pin:{
      if(checkDebounce(Caixa_NivelAlto_Pin, GPIOA) == 1){
        uint8_t paradaTipo = 1;
        xQueueSendToFrontFromISR(Queue_StopMotor, &paradaTipo, &xHigherPriorityTaskWoken);
      }
    }
    break;
    case ligarMotor1min_Pin:{
      if(checkDebounce(ligarMotor1min_Pin, GPIOA) == 1){
        uint8_t tempoMotor = 1;
        xQueueSendToFrontFromISR(Queue_StartMotor, &tempoMotor, &xHigherPriorityTaskWoken);
      }
    }
    break;
    case ligarMotor5min_Pin:{
      if(checkDebounce(ligarMotor5min_Pin, GPIOA) == 1){
        uint8_t tempoMotor = 5;
        xQueueSendToFrontFromISR(Queue_StartMotor, &tempoMotor, &xHigherPriorityTaskWoken);
      }
    }
    break;
    case ligarMotor10min_Pin:{
      if(checkDebounce(ligarMotor10min_Pin, GPIOA) == 1){
        uint8_t tempoMotor = 10;
        xQueueSendToFrontFromISR(Queue_StartMotor, &tempoMotor, &xHigherPriorityTaskWoken);
        return;
      }
      return;
    }
    break;
  }
}

int checkDebounce(uint16_t GPIO_Pin, GPIO_TypeDef  *GPIO_Port){
 int inputLow = 0;
  for(int i=0;i<1000000;i++){
    if(HAL_GPIO_ReadPin(GPIO_Port,GPIO_Pin)==0){
      inputLow++;
    }
  }

  if(inputLow < 15){
    return 1;
  }
  return 0;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
char* msg1 = "1 Aguardando Nivel Alto Poco\r\n";
char* msg2 = "2 Aguardando Nivel Baixo Caixa\r\n";
char* msg3 = "3 Ligando Motor\r\n";
char* msg4 = "4.1 Caixa_NivelAlto\r\n";
char* msg5 = "4.2 Poco_NivelBaixo\r\n";
char* msg6 = "5 Btn Para Ligar Motor acionado\r\n";
char* msg7 = "6 Btn Para Ligar Motor fim Execucao\r\n";
char* msgTempoMaximoMotor = "7 Tempo maximo de motor ligado excedido\r\n";
char* newline = "\r\n\r\n";
int print = 1;
int tempoMaximoMotorLigado = 15;
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  printf("Iniciando...\n");
  /* USER CODE BEGIN 5 */
  uint8_t tipoParada;
  int delayLedAzul = 1000;
  /* Infinite loop */
  for(;;)
  {
    vTaskSuspend(taskMotorLigadoHandle);
    HAL_GPIO_WritePin(GPIOB, Motor_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LedAzul_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LedePCI_GPIO_Port, LedePCI_Pin, GPIO_PIN_RESET);
    while(uxQueueMessagesWaiting( Queue_StopMotor ) != 0){
      xQueueReceive(Queue_StopMotor, &tipoParada, 0);
      vTaskDelay(10);
    }
    //1 Nivel Alto Poco

    if(print == 1){
      CDC_Transmit_FS(newline, strlen(newline));
      CDC_Transmit_FS(msg1, strlen(msg1));
      printf("%s", msg1);
    }
    int tempoParaApagarLeds = 60 * 5;//Minutos
    int counter = 0;
    while(HAL_GPIO_ReadPin(GPIOA,Poco_NivelAlto_Pin)==0){
      vTaskDelay(1000);
      HAL_GPIO_WritePin(LedePCI_GPIO_Port, LedePCI_Pin, GPIO_PIN_RESET);
      vTaskDelay(1000);
      HAL_GPIO_WritePin(LedePCI_GPIO_Port, LedePCI_Pin, GPIO_PIN_SET);
      counter++;
      if(counter >= tempoParaApagarLeds){
        counter = 0;
        HAL_GPIO_WritePin(GPIOB, Motor_Pin|LedAzul_Pin|LedAmarelo_Pin|LedVerde_Pin, GPIO_PIN_RESET);
        while(uxQueueMessagesWaiting( Queue_StopMotor ) != 0){
          xQueueReceive(Queue_StopMotor, &tipoParada, 0);
          vTaskDelay(10);
        }
      }
      print = 1;
    }


    HAL_GPIO_WritePin(LedePCI_GPIO_Port, LedePCI_Pin, GPIO_PIN_SET);
    if(print == 1){
      CDC_Transmit_FS(msg2, strlen(msg2));
      printf("%s", msg2);
      print = 0;
    }
    vTaskDelay(1000);
    if(HAL_GPIO_ReadPin(GPIOA,Caixa_NivelBaixo_Pin)==0){
        print = 1;
        CDC_Transmit_FS(msg3, strlen(msg3));
        printf("%s", msg3);
        HAL_GPIO_WritePin(GPIOB, Motor_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, LedAzul_Pin, GPIO_PIN_SET);

        xQueueSend(Queue_delayLedAzul, &delayLedAzul, 10);
        vTaskResume(taskMotorLigadoHandle);

        HAL_GPIO_WritePin(GPIOB, LedAmarelo_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LedVerde_Pin, GPIO_PIN_RESET);
        for(;;){
           int tempoMotorLigado = 0;
           while(uxQueueMessagesWaiting( Queue_StopMotor ) == 0 && tempoMotorLigado < (tempoMaximoMotorLigado * 60)){
             vTaskDelay(1000);
             tempoMotorLigado++;
           }

           if(tempoMotorLigado >= (tempoMaximoMotorLigado * 60)){
             CDC_Transmit_FS(msgTempoMaximoMotor, strlen(msg4));
             printf("%s", msgTempoMaximoMotor);
             printf("%s", msg4);
             HAL_GPIO_WritePin(GPIOB, LedVerde_Pin, GPIO_PIN_SET);
             HAL_GPIO_WritePin(GPIOB, LedAmarelo_Pin, GPIO_PIN_SET);
             break;
           }
           else {
              xQueueReceive(Queue_StopMotor, &tipoParada, 0);

              //Caixa Cheia
              if(tipoParada==1){
                CDC_Transmit_FS(msg4, strlen(msg4));
                printf("%s", msg4);
                HAL_GPIO_WritePin(GPIOB, LedVerde_Pin, GPIO_PIN_SET);
                break;
              }

              //Poco Vazio
              if(tipoParada == 2){
                CDC_Transmit_FS(msg5, strlen(msg5));
                printf("%s", msg5);
                HAL_GPIO_WritePin(GPIOB, LedAmarelo_Pin, GPIO_PIN_SET);
                break;
              }
           }
          //printf("4.3 Enchendo a caixa...");
        }
      }


    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the taskMotorLigado thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  int delayLedAzul = 1000;
  for(;;)
  {
    while(uxQueueMessagesWaiting( Queue_delayLedAzul ) != 0){
        xQueueReceive(Queue_delayLedAzul, &delayLedAzul, 0);
        vTaskDelay(1);
    }

    HAL_GPIO_WritePin(GPIOB, LedAzul_Pin, GPIO_PIN_SET);
    vTaskDelay(delayLedAzul);
    HAL_GPIO_WritePin(GPIOB, LedAzul_Pin, GPIO_PIN_RESET);
    vTaskDelay(delayLedAzul);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_ligarMotor */
/**
* @brief Function implementing the btnLigarMtr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ligarMotor */
void ligarMotor(void const * argument)
{
  /* USER CODE BEGIN ligarMotor */
  /* Infinite loop */
  uint8_t tempoMotorLigado;
  for(;;)
  {
    while(uxQueueMessagesWaiting( Queue_StartMotor ) == 0){
       vTaskDelay(500);
    }

    while(uxQueueMessagesWaiting( Queue_StartMotor ) != 0){
      xQueueReceive(Queue_StartMotor, &tempoMotorLigado, 0);
      vTaskDelay(10);
   }

    vTaskSuspend(defaultTaskHandle);
    CDC_Transmit_FS(msg6, strlen(msg6));
    printf("%s", msg6);
    HAL_GPIO_WritePin(GPIOB, Motor_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LedAzul_Pin, GPIO_PIN_SET);

    int delayLedAzul = 100;
    switch(tempoMotorLigado){
      case 1:
        delayLedAzul = 200;
        break;
      case 5:
        delayLedAzul = 500;
        break;
      case 10:
        delayLedAzul = 700;
        break;
    }
    xQueueSend(Queue_delayLedAzul, &delayLedAzul, 10);
    vTaskResume(taskMotorLigadoHandle);
    int tempoMotorLigadoTemp = tempoMotorLigado * 60; //10 minutos
    for(int i=0;i<tempoMotorLigadoTemp*2;i++){
      if(uxQueueMessagesWaiting( Queue_StopMotor ) != 0){
        uint8_t tipoParada = 0;
        xQueueReceive(Queue_StopMotor, &tipoParada, 0);
        //Poco Vazio
        if(tipoParada == 2){
          CDC_Transmit_FS(msg5, strlen(msg5));
          printf("%s", msg5);
          HAL_GPIO_WritePin(GPIOB, LedAmarelo_Pin, GPIO_PIN_SET);
          break;
        }
      }
      vTaskDelay(500);
    }
    vTaskSuspend(taskMotorLigadoHandle);
    HAL_GPIO_WritePin(GPIOB, Motor_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LedAzul_Pin, GPIO_PIN_RESET);
    CDC_Transmit_FS(msg7, strlen(msg7));
    printf("%s", msg7);

    while(uxQueueMessagesWaiting( Queue_StartMotor ) != 0){
      xQueueReceive(Queue_StartMotor, &tempoMotorLigado, 0);
      vTaskDelay(10);
   }
    vTaskResume(defaultTaskHandle);
  }
  /* USER CODE END ligarMotor */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
