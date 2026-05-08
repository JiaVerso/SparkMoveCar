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
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rtthread.h>
#include <stdint.h>
#include "fifo.h"
#include <string.h>
#include <math.h>

#include "drv_can.h"
#include "drv_bsp.h"
#include "drv_uart.h"
#include "motor_dji.h"
#include "serial_plotter.h"
#include "pidcontroller.h"
#include "app_test.h"
#include "drv_math.h"
#include "dev_dm4310.h"
#include "drv_dm4310.h"
#include "drv_imu.h"
#include "QuaternionEKF.h" 
#include "dev_dm_imu.h"

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
float Now_Omega, Target_Omega = 350.0f * PI;
uint32_t Counter = 0;
uint8_t speed_state = 0;

// 实例化对象
SerialPlotter_t my_plotter;
Motor_t motor_ID1;
PID_t  pid_speed;     // 速度环
extern dm_motor_t motor[Motor_Max];

int8_t gyro_map[3]  = {1, 2, 3};
int8_t accel_map[3] = {1, 2, 3};

typedef union {
    float f_data[4];      // 3个数据 + 1个包尾 = 4个float
    uint8_t byte_data[16]; // 4 * 4 = 16 字节
} VOFA_JustFloat_t;

VOFA_JustFloat_t vofa_packet;
extern imu_t imu;

void VOFA_Init(void) {
    vofa_packet.byte_data[12] = 0x00;
    vofa_packet.byte_data[13] = 0x00;
    vofa_packet.byte_data[14] = 0x80;
    vofa_packet.byte_data[15] = 0x7F;
}
/**
 * @brief 串口 DMA 循环绕回处理核心算法 (类的方法)
 * @param obj 串口管理对象指针
 * @param Size DMA 当前写到的位置 (HAL库传入)
 */
void UART8_Trigger_Tx_DMA(void)
{
    App_Test_Trigger_UART_DMA(&huart8, uart_rx_fifo, tx_dma_buf, TX_DMA_BUF_SIZE);
}

void Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    App_Test_Parse_Command(Buffer, Length);
}

 /* ---------------------------------------CAN Callback Configuration--------------------------------------------------------*/
/**
 * @brief  滤波器在CAN总线上获取到目标ID触发中断
 * @param  hcan CAN编号
 * @param  header  Rx接收头
 * @param  HAL_CAN_GetRxMessage HAL库内部函数-从FIFO中获取数据帧
 * @retval None
 */
void Motor_Cmd_TxCallback(Struct_CAN_Rx_Buffer *Rx_Buffer) {
  // 暂时未处理电调发送过来的反馈信息
  uint8_t *Rx_Data = Rx_Buffer->Data;
  uint32_t id = Rx_Buffer->Header.StdId;
  switch (id) {
  case (0x23): {
    IMU_UpdateData(Rx_Data);
  } break;
  default:
    break;
  }
}

void UART8_Send_To_Plotter_DMA(uint8_t *data, uint16_t len) {
    // 调用 HAL 库的 DMA 发送函数
    UART_Send_Data(&huart8, data, 1 + 12 * sizeof(float));
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motor_thread_entry(void *parameter)
{
  while (1)
  {
    Counter++;
    pos_speed_ctrl(&hcan1, motor[Motor1].id, (Counter / 100) % 2 == 0 ? 0.0f : PI, PI);

    rt_thread_mdelay(10);
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
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */

  BSP_Init(BSP_DC_LU_ON | BSP_DC_LD_ON | BSP_DC_RU_ON | BSP_DC_RD_ON | BSP_LED_RED_ON, 0, 0);
  
  Uart_Init(&huart8, rx_buffer, 128, Serialplot_Call_Back);
  // PID_Init(&pid_speed, 0.0f, 0.0f, 0.0f, 0.0f,2500.0f, 2500.0f);

  // dm4310_motor_init(&hcan1, &motor[Motor1], MOTOR_LEFT_CANID, POS_MODE);
  // // DM-BMI088初始化

  // uart_rx_fifo = fifo_s_create(2048);
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart8, USART8_Rx_buf, RX_BUF_SIZE);

  // Motor_Init(&motor_ID1, 0x202, 0x200);
  // MotorManager_Register(&motor_ID1);
  // Plotter_Init(&my_plotter, rx_buffer, 0xAA, UART8_Send_To_Plotter_DMA);
  
  // ctrl_enable(Motor1_Status_ENABLED);
  CAN_Filter_Mask_Config(
      &hcan1, CAN_FILTER(13) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
  CAN_Init(&hcan1, Motor_Cmd_TxCallback);
  VOFA_Init();

  imu_init(0x22, 0x23, &hcan1);

  // imu_change_to_active();       
  // imu_save_parameters();        

  // imu_change_to_active();
  /* 创建并启动电机控制线程 (优先级 15) */
  // rt_thread_t motor_tid = rt_thread_create("motor", motor_thread_entry, RT_NULL, 1024, 15, 10);
  // rt_thread_startup(motor_tid);

  // mpu_device_init();

  // IMU_QuaternionEKF_Init(10.0f, 0.001f, 1.0e7f, 1.0f, 0.01f);
  // IMU_QuaternionEKF_Set_MPU6500_Config(16.384f, 4096.0f, 9.80665f, gyro_map, accel_map);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Plotter_Append(&my_plotter, motor_ID1.rx_angle);
    // Plotter_Append(&my_plotter, Target_Omega);
    // Plotter_Append(&my_plotter, current_speed_float);
    // Plotter_Append(&my_plotter, pid_speed.Kp);
    // Plotter_Append(&my_plotter, motor_ID1.rx_torque);
    // Plotter_Append(&my_plotter, motor_ID1.rx_temperature);

    // Plotter_Begin(&my_plotter);
    // Plotter_Append(&my_plotter, motor[Motor1].para.pos);
    // Plotter_Append(&my_plotter, motor[Motor1].para.vel);
    // Plotter_Append(&my_plotter, motor[Motor1].para.tor);
    // Plotter_Append(&my_plotter, motor[Motor1].para.Tmos);
    // Plotter_SendData(&my_plotter);

    // mpu_get_data();
    // HAL_UART_Transmit_DMA(&huart8, (const uint8_t *)&imu, sizeof(imu_t));

    // rt_thread_mdelay(20);

    // float dt = 0.001f; 
    // mpu_get_data();

    //     IMU_QuaternionEKF_Update_MPU6500_Raw(
    //         mpu_data.gx, mpu_data.gy, mpu_data.gz,
    //         mpu_data.ax, mpu_data.ay, mpu_data.az,
    //         dt
    //     );
    // HAL_Delay(10);
    if (huart8.gState == HAL_UART_STATE_READY)
{
    vofa_packet.f_data[0] = imu.roll;
    vofa_packet.f_data[1] = imu.pitch;
    vofa_packet.f_data[2] = imu.yaw;
    HAL_UART_Transmit_DMA(&huart8, vofa_packet.byte_data, 16);
}
HAL_Delay(20);


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

