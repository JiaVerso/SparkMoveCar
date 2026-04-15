/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rtthread.h>
#include <stdint.h>
#include "fifo.h"
#include <string.h>

#include <drv_can.h>
#include <drv_bsp.h>
#include <drv_uart.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUF_SIZE 1024
uint8_t USART8_Rx_buf[RX_BUF_SIZE];   // DMA Buff

#define TX_DMA_BUF_SIZE 2048
static uint8_t tx_dma_buf[TX_DMA_BUF_SIZE];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
fifo_s_t *uart_rx_fifo = NULL;
uint8_t rx_buffer[128];
/**
 * @brief 串口 DMA 循环绕回处理核心算法 (类的方法)
 * @param obj 串口管理对象指针
 * @param Size DMA 当前写到的位置 (HAL库传入)
 */
void UART8_Trigger_Tx_DMA(void)
{
    // 1. 检查当前串口的 TX 状态是否空闲 (非常重要！)
    // 如果 DMA 正在搬运上一批数据，千万不能打断它，直接退出
    if (huart8.gState == HAL_UART_STATE_READY) 
    {
        // 2. 看看 FIFO 里有多少待发送的数据
        uint16_t len = fifo_s_used(uart_rx_fifo); 
        
        if (len > 0) 
        {
            // 防溢出保护：如果 FIFO 里的数据比我的缓冲区还大，这次只取缓冲区的上限
            if (len > TX_DMA_BUF_SIZE) 
            {
                len = TX_DMA_BUF_SIZE;
            }
            
            // 3. 把数据从 FIFO 拿出来放到物理缓冲区
            fifo_s_gets(uart_rx_fifo, (char *)tx_dma_buf, len);
            
            // 4. 开启 DMA 搬运！
            HAL_UART_Transmit_DMA(&huart8, tx_dma_buf, len);
        }
    }
}

void Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    if (rx_buffer[0] == '0')
    {
        BSP_LED_1(BSP_LED_Status_DISABLED);
    }
    else if (rx_buffer[0] == '1')
    {
        BSP_LED_1(BSP_LED_Status_ENABLED);
    }
    else if (rx_buffer[0] == '2')
    {
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
    }
}

 /* ---------------------------------------CAN Callback Configuration--------------------------------------------------------*/
/**
  * @brief  滤波器在CAN总线上获取到目标ID触发中断
  * @param  hcan CAN编号
  * @param  header  Rx接收头
  * @param  HAL_CAN_GetRxMessage HAL库内部函数-从FIFO中获取数据帧
  * @retval None
  */
void Motor_Cmd_TxCallback(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
  // 暂时未处理电调发送过来的反馈信息
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void my_task_entry(void *parameter)
{   
  
    // uart_rx_fifo = fifo_s_create(1024);

    // HAL_UARTEx_ReceiveToIdle_DMA(&huart7, USART7_Rx_buf, RX_BUF_SIZE);

    while (1)
    {     	
      // if (huart7.gState == HAL_UART_STATE_READY) 
      //   {
      //       HAL_UART_Transmit_DMA(&huart7, tx_temp_buf, 5);
      //   }
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      HAL_Delay(500);
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      HAL_Delay(500);
      //   // 必须加延时，否则 RT-Thread 的其他低优先级任务（如 idle）会被饿死
      //   rt_thread_mdelay(5);
    }
}

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
  MX_DMA_Init();
  MX_UART8_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  BSP_Init(BSP_DC_LU_ON | BSP_DC_LD_ON | BSP_DC_RU_ON | BSP_DC_RD_ON | BSP_LED_GREEN_ON, 0, 0);
  CAN_Init(&hcan1, Motor_Cmd_TxCallback);
  Uart_Init(&huart8, rx_buffer, 10, Serialplot_Call_Back);

  uart_rx_fifo = fifo_s_create(2048);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart8, USART8_Rx_buf, RX_BUF_SIZE);

  // uint8_t Send_Data = 0;
 
  CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(13) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x200, 0x7ff);
  

  // rt_thread_t tid = rt_thread_create("my_task", my_task_entry, RT_NULL, 1024, 15, 10);
  // if (tid != RT_NULL)
  //   {
  //       rt_thread_startup(tid);
  //   }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  static uint32_t flag;
  UART8_Tx_Data[0] = 0xAA;
  float tmp_data;

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    if (flag == 2500) {
      flag = 0;
    }
    
    tmp_data = ((float)flag / 1000.0f) * ((float)flag / 1000.0f);
    for (uint8_t i = 0; i < 4; i++) {
      UART8_Tx_Data[i + 1] = *((char *)(&tmp_data) + i);
    }

    float led_status;
    led_status = !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1);
    for (uint8_t i = 0; i < 4; i++) {
      UART8_Tx_Data[i + 5] = *((char *)(&led_status) + i);
    }

    HAL_Delay(0);
    flag++;
    UART_Send_Data(&huart8, UART8_Tx_Data, 9);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  if (htim->Instance == TIM1)
  {
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
