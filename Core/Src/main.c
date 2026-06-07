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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rtthread.h>
#include <stdint.h>
#include "fifo.h"
#include <string.h>
#include <math.h>

#include "usart.h"
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
#include "dev_n630.h"
#include "bsp_sbus.h"
#include "chassis.h"
#include "uxrce_transport_uart.h"
#include "uxrce_app.h"
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
static uint8_t uxrce_rx_dma_buf[256];
static uint8_t debug_rx_dma_buf[256];


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
N630_Motor_t n630_motor[30] = {0};

int8_t gyro_map[3]  = {1, 2, 3};
int8_t accel_map[3] = {1, 2, 3};

typedef union {
    float f_data[4];      // 3个数据 + 1个包尾 = 4个float
    uint8_t byte_data[16]; // 4 * 4 = 16 字节
} VOFA_JustFloat_t;

VOFA_JustFloat_t vofa_packet;
extern imu_t imu;
extern uint8_t sbus_dma_buf[25];
extern ChassisMotor_t ChassisMotor_Table[CHASSIS_MOTOR_COUNT];

void VOFA_Init(void) {
    vofa_packet.byte_data[48] = 0x00;
    vofa_packet.byte_data[49] = 0x00;
    vofa_packet.byte_data[50] = 0x80;
    vofa_packet.byte_data[51] = 0x7F;
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
    for (uint16_t i = 0; i < Length; i++)
    {
        SBUS_Receive(Buffer[i]);
    }

    SBUS_Handle();
}

void Serialplot8_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    App_Test_Parse_Command(Buffer, Length);
}

void SerialDebug_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    // Implementation for parsing commands
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
  ChassisMotor_CANRxDispatch(Rx_Buffer);
}

void DMotor_Cmd_TxCallback(Struct_CAN_Rx_Buffer *Rx_Buffer) {
  // 解析 DM-J4310 的反馈 
  ChassisMotor_CANRxDispatch(Rx_Buffer);
}

void UART8_Send_To_Plotter_DMA(uint8_t *data, uint16_t len) {
    // 调用 HAL 库的 DMA 发送函数
    UART_Send_Data(&huart8, data, len);
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
  MX_USART1_UART_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */

  BSP_Init(BSP_DC_LU_ON | BSP_DC_LD_ON | BSP_DC_RU_ON | BSP_DC_RD_ON | BSP_LED_GREEN_ON, 0, 0);
  
  // SBUS DMA 初始化
  Uart_Init(&huart1, sbus_dma_buf, 25, Serialplot_Call_Back);
  // Uart_Init(&huart8, rx_buffer, RX_BUF_SIZE, Serialplot8_Call_Back);

  // Micro XRCE DDS 
  Uart_Init(&huart8, uxrce_rx_dma_buf, sizeof(uxrce_rx_dma_buf), uxrDds_UartRxCallback);

  // DEBUG UART
  Uart_Init(&huart7, debug_rx_dma_buf, sizeof(debug_rx_dma_buf), SerialDebug_Call_Back);

  uxrce_app_init();

  // PID_Init(&pid_speed, 0.0f, 0.0f, 0.0f, 0.0f,2500.0f, 2500.0f);

  // uart_rx_fifo = fifo_s_create(2048);
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart8, USART8_Rx_buf, RX_BUF_SIZE);

  // Plotter_Init(&my_plotter, rx_buffer, 0xAA, UART8_Send_To_Plotter_DMA);
  
  CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
  CAN_Filter_Ext_Mask_Config(&hcan1, 1, 0, 0, CAN_FILTER_FIFO1);
  CAN_Init(&hcan1, Motor_Cmd_TxCallback);

  CAN_Filter_Mask_Config(&hcan2, CAN_FILTER(27) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
  CAN_Filter_Ext_Mask_Config(&hcan2, 26, 0, 0, CAN_FILTER_FIFO1);
  CAN_Init(&hcan2, DMotor_Cmd_TxCallback);
  
  dm4310_motor_init(&hcan2, &motor[Motor1], MOTOR_LEFT_CANID, 1);
  dm4310_motor_init(&hcan2, &motor[Motor2], MOTOR_RIGHT_CANID, 1);
  ctrl_enable(Motor_ALL_Status_ENABLED);
  HAL_Delay(200);
  // save_pos_zero(&hcan2, MOTOR_LEFT_CANID, POS_MODE);
  // save_pos_zero(&hcan2, MOTOR_RIGHT_CANID, POS_MODE);
  // HAL_Delay(500);

  // VOFA_Init();
  // ChassisMotor_InitAll();
  

  // imu_init(0x22, 0x23, &hcan1);

  // imu_change_to_active();       
  // imu_save_parameters();        

  // imu_change_to_active();
  /* 创建并启动电机控制线程 (优先级 15) */
  // rt_thread_t motor_tid = rt_thread_create("motor", motor_thread_entry, RT_NULL, 1024, 15, 10);
  // rt_thread_startup(motor_tid);

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

    // float rpm = n630_motor[21].rpm;
    // HAL_UART_Transmit_DMA(&huart8, (const uint8_t *)&rpm, sizeof(float));
    // comm_can_set_rpm(21, 8000.0f);
    // comm_can_set_rpm(22, 8000.0f);
    // comm_can_set_rpm(25, 8000.0f);
    // comm_can_set_rpm(&hcan1, 26, 4000.0f);
  
    // ChassisMotor_ControlLoop();

    // pos_speed_ctrl(&hcan2, MOTOR_LEFT_CANID, 0.0f, 1.0f);
    // pos_speed_ctrl(&hcan2, MOTOR_RIGHT_CANID, 0.0f, 1.0f);

    // Plotter_Begin(&my_plotter);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[0].target_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[0].feedback_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[0].current_cmd);

    // Plotter_Append(&my_plotter, ChassisMotor_Table[1].target_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[1].feedback_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[1].current_cmd);

    // Plotter_Append(&my_plotter, ChassisMotor_Table[2].target_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[2].feedback_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[2].current_cmd);

    // Plotter_Append(&my_plotter, ChassisMotor_Table[3].target_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[3].feedback_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[3].current_cmd);

    // Plotter_Append(&my_plotter, ChassisMotor_Table[4].target_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[4].feedback_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[4].current_cmd);

    // Plotter_Append(&my_plotter, ChassisMotor_Table[5].target_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[5].feedback_wheel_rpm);
    // Plotter_Append(&my_plotter, ChassisMotor_Table[5].current_cmd);
    // static uint32_t counter = 0;
    // if(counter++ % 500 == 0) {
    //      char msg[256];
    //     // 发送SBUS数据到串口，方便调试和监控  Send SBUS data to the serial port for debugging and monitoring
    //     // 这里是CH1~CH8的数据，便于调试添加命名
    //     int len = snprintf(msg, sizeof(msg),
    //                "1_T:%ld 1_R:%ld 1_C:%ld\r\n 2_T:%ld 2_R:%ld 2_C:%ld\r\n 3_T:%ld 3_R:%ld 3_C:%ld\r\n 4_T:%ld 4_R:%ld 4_C:%ld\r\n 5_T:%ld 5_R:%ld 5_C:%ld\r\n 6_T:%ld 6_R:%ld 6_C:%ld\r\n",
    //                (long)ChassisMotor_Table[0].target_wheel_rpm, (long)ChassisMotor_Table[0].feedback_wheel_rpm, (long)ChassisMotor_Table[0].current_cmd,
    //                (long)ChassisMotor_Table[1].target_wheel_rpm, (long)ChassisMotor_Table[1].feedback_wheel_rpm, (long)ChassisMotor_Table[1].current_cmd,
    //                (long)ChassisMotor_Table[2].target_wheel_rpm, (long)ChassisMotor_Table[2].feedback_wheel_rpm, (long)ChassisMotor_Table[2].current_cmd,
    //                (long)ChassisMotor_Table[3].target_wheel_rpm, (long)ChassisMotor_Table[3].feedback_wheel_rpm, (long)ChassisMotor_Table[3].current_cmd,
    //                (long)ChassisMotor_Table[4].target_wheel_rpm, (long)ChassisMotor_Table[4].feedback_wheel_rpm, (long)ChassisMotor_Table[4].current_cmd,
    //                (long)ChassisMotor_Table[5].target_wheel_rpm, (long)ChassisMotor_Table[5].feedback_wheel_rpm, (long)ChassisMotor_Table[5].current_cmd);

    //     UART_Send_Data(&huart8, (uint8_t *)msg, len);
    //     counter = 0;
    // }

    // Plotter_SendData(&my_plotter);
    uxrce_app_loop();

    HAL_Delay(500);
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
