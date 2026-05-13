#include "bsp_sbus.h"
#include "string.h"
#include "stdio.h"
#include "driver_motor.h"


#define SBUS_RECV_MAX    25
#define SBUS_START       0x0F
#define SBUS_END         0x00
#define EMERGENCY_STOP   1000
#define CAR_LENGTH       12.5
#define CAR_WIDTH        18.5
#define MANUAL 0x00

// SBUS DMA buffer  SBUS DMA缓冲区
uint8_t sbus_dma_buf[25] = {0};

// Parameters related to receiving data  接收数据相关参数
static uint8_t sbus_start = 0;
static uint8_t sbus_buf_index = 0;

//  Indicates whether a new SBUS command has been received  表示是否接收到新的SBUS命令
static uint8_t sbus_new_cmd = 0;

// data-caching mechanism  数据缓存
static uint8_t inBuffer[SBUS_RECV_MAX] = {0};
static uint8_t failsafe_status = SBUS_SIGNAL_FAILSAFE;

static uint8_t sbus_data[SBUS_RECV_MAX] = {0};
static uint16_t g_sbus_channels[18] = {0};

// SBUS DMA start function  SBUS DMA启动函数
void SBUS_DMA_Start(UART_HandleTypeDef *huart)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, sbus_dma_buf, sizeof(sbus_dma_buf));

    // Disable the half-transfer interrupt to avoid unnecessary interrupts  禁止半传输中断，避免不必要的中断
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

// Receives SBUS cache data  接收SBUS的缓存数据
void SBUS_Receive(uint8_t data)
{
    // 如果符合协议开始标志，则开始接收数据
    if (sbus_start == 0 && data == SBUS_START)
    {
        sbus_start = 1;
        sbus_new_cmd = 0;
        sbus_buf_index = 0;
        inBuffer[sbus_buf_index] = data;          // 起始位
        inBuffer[SBUS_RECV_MAX - 1] = 0xff;       // 接收一帧数据后，判断停止位是否正确
    }
    else if (sbus_start)
    {
        sbus_buf_index++;
        inBuffer[sbus_buf_index] = data;
    }

    // Finish receiving a frame of data  完成接收一帧数据
    if (sbus_start && (sbus_buf_index >= (SBUS_RECV_MAX - 1)))
    {
        sbus_start = 0;
        if (inBuffer[SBUS_RECV_MAX - 1] == SBUS_END)
        {
            memcpy(sbus_data, inBuffer, SBUS_RECV_MAX);
            sbus_new_cmd = 1;
        }
    }
}

// Parses SBUS data into channel values  解析SBUS的数据，转化成通道数值。
static int SBUS_Parse_Data(void)
{
    g_sbus_channels[0]  = ((sbus_data[1] | sbus_data[2] << 8) & 0x07FF);
    g_sbus_channels[1]  = ((sbus_data[2] >> 3 | sbus_data[3] << 5) & 0x07FF);
    g_sbus_channels[2]  = ((sbus_data[3] >> 6 | sbus_data[4] << 2 | sbus_data[5] << 10) & 0x07FF);
    g_sbus_channels[3]  = ((sbus_data[5] >> 1 | sbus_data[6] << 7) & 0x07FF);
    g_sbus_channels[4]  = ((sbus_data[6] >> 4 | sbus_data[7] << 4) & 0x07FF);
    g_sbus_channels[5]  = ((sbus_data[7] >> 7 | sbus_data[8] << 1 | sbus_data[9] << 9) & 0x07FF);
    g_sbus_channels[6]  = ((sbus_data[9] >> 2 | sbus_data[10] << 6) & 0x07FF);
    g_sbus_channels[7]  = ((sbus_data[10] >> 5 | sbus_data[11] << 3) & 0x07FF);
    #ifdef ALL_CHANNELS
    g_sbus_channels[8]  = ((sbus_data[12] | sbus_data[13] << 8) & 0x07FF);
    g_sbus_channels[9]  = ((sbus_data[13] >> 3 | sbus_data[14] << 5) & 0x07FF);
    g_sbus_channels[10] = ((sbus_data[14] >> 6 | sbus_data[15] << 2 | sbus_data[16] << 10) & 0x07FF);
    g_sbus_channels[11] = ((sbus_data[16] >> 1 | sbus_data[17] << 7) & 0x07FF);
    g_sbus_channels[12] = ((sbus_data[17] >> 4 | sbus_data[18] << 4) & 0x07FF);
    g_sbus_channels[13] = ((sbus_data[18] >> 7 | sbus_data[19] << 1 | sbus_data[20] << 9) & 0x07FF);
    g_sbus_channels[14] = ((sbus_data[20] >> 2 | sbus_data[21] << 6) & 0x07FF);
    g_sbus_channels[15] = ((sbus_data[21] >> 5 | sbus_data[22] << 3) & 0x07FF);
    #endif

    // 安全检测，检测是否失联或者数据错误
    // Security detection to check for lost connections or data errors
    failsafe_status = SBUS_SIGNAL_OK;
    if (sbus_data[23] & (1 << 2))
    {
        failsafe_status = SBUS_SIGNAL_LOST;
        printf("SBUS_SIGNAL_LOST\n");
        // lost contact errors  遥控器失联错误
    }
    else if (sbus_data[23] & (1 << 3))
    {
        failsafe_status = SBUS_SIGNAL_FAILSAFE;
        printf("SBUS_SIGNAL_FAILSAFE\n");
        // data loss error  数据丢失错误
    }
    return failsafe_status;
}

//  SBUS接收处理数据
void SBUS_Handle(void)
{
    if (sbus_new_cmd)
    {
        int res = SBUS_Parse_Data();
        sbus_new_cmd = 0;
        if (res) return;

        #if SBUS_ALL_CHANNELS
        
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               g_sbus_channels[0], g_sbus_channels[1], g_sbus_channels[2],
			   g_sbus_channels[3], g_sbus_channels[4], g_sbus_channels[5],
			   g_sbus_channels[6], g_sbus_channels[7], g_sbus_channels[8],
			   g_sbus_channels[9], g_sbus_channels[10], g_sbus_channels[11],
			   g_sbus_channels[12], g_sbus_channels[13], g_sbus_channels[14],
			   g_sbus_channels[15]);
        #else

        char msg[128];
        // 发送SBUS数据到串口，方便调试和监控  Send SBUS data to the serial port for debugging and monitoring
        // 这里是CH1~CH8的数据，便于调试添加命名
        int len = snprintf(msg, sizeof(msg),
                   "roll:%u pitch:%u thro:%u yaw:%u sw_c:%u roller:%u back_1:%u roller_2:%u\r\n",
                   g_sbus_channels[0], g_sbus_channels[1],
                   g_sbus_channels[2], g_sbus_channels[3],
                   g_sbus_channels[4], g_sbus_channels[5],
                   g_sbus_channels[6], g_sbus_channels[7]);

        UART_Send_Data(&huart8, (uint8_t *)msg, len);
        // 将SBUS协议转换成PWM，传递给电机控制函数  The SBUS protocol is converted to PWM and passed to the motor control function
        DriveMotor_UpdateFromSbusChannels(g_sbus_channels);

        #endif
    }
}

// // The SBUS protocol is converted to PWM and passed to the motor control function
// void Linear_Mapping(uint16_t g_sbus_channels[]) {
//     sbus_data_t sbus;
//     if (g_sbus_channels[0] == 0 && g_sbus_channels[2] == 0) {
//         move.vx = move.vy = move.vz = 0.0f;
//         for (int i = 0 ; i<4 ;i++) {
//             motor_spee_dlinera[i] = 0;
//         }
//         return;
//     }

#if  MANUAL
    move.vy = ((g_sbus_channels[0] - 1000) / 820.0f);  // 横滚
    move.vx = ((g_sbus_channels[1] - 1000) / 820.0f);  //俯仰
    move.throttle = (g_sbus_channels[2] - 1000) / 820.0f;   // 设置 SPEED 大小
    move.vz = ((g_sbus_channels[3] - 980) / 820.0f);   // 偏航

    if (fabs(move.vy) < 0.10f)  move.vy = 0.0f;
    if (fabs(move.vx) < 0.10f)  move.vx = 0.0f;
    if (fabs(move.throttle) < 0.10f)  move.throttle = 0.0f;
    if (fabs(move.vz) < 0.10f)  move.vz = 0.0f;

    // 限幅
    motor_speed_linera[0] = move.vx - move.vy - move.vz * K;
    motor_speed_linera[1] = move.vx + move.vy - move.vz * K;
    motor_speed_linera[2] = move.vx + move.vy + move.vz * K;
    motor_speed_linera[3] = move.vx - move.vy + move.vz * K;

    float max_linera = 1.0f;
    for (int i = 0; i < 4; i++) {
        if (fabs(motor_speed_linera[i]) > 1.0f)  max_linera = fabs(motor_speed_linera[i]);
    }
#else
    // int max_motor = 500;
    // move.vy = ((g_sbus_channels[0] - 1020) / 820.0f) * max_motor;  // 横滚
    // move.vx = ((g_sbus_channels[1] - 990) / 820.0f) * max_motor;  //俯仰
    // move.vz = ((g_sbus_channels[3] - 980) / 820.0f) * max_motor;   // 偏航

    // if (fabs(move.vy) < 50.0f)  move.vy = 0.0f;
    // if (fabs(move.vx) < 50.0f)  move.vx = 0.0f;
    // if (fabs(move.vz) < 50.0f)  move.vz = 0.0f;

    // Motion_Ctrl(move.vx, move.vy, move.vz);

#endif

