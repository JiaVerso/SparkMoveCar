#include "uxrce_app.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include <stdbool.h>
#include <uxr/client/client.h>
#include "gpio.h"

#include <string.h>
#include <ucdr/microcdr.h>
#include <uxr/client/util/ping.h>
#include <stdio.h>

#include "uxrce_pub_topics.h"
#include "uxrce_sub_ackermann.h"

#define STREAM_HISTORY  8
#define BUFFER_SIZE     2048

uxrCustomTransport transport;

// uart_args.huart = &huart8
uxrUartTransArgs_t uart_args = {
    .huart = &huart8,
};

static uxrSession session;
static uxrStreamId reliable_out;
static uxrStreamId reliable_in;
static uxrObjectId participant_id;

static volatile bool session_ok =false;

static uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
static uint8_t input_reliable_stream_buffer[BUFFER_SIZE];

void uxrce_blink(GPIO_TypeDef* GPIO, uint16_t GPIO_Pin, uint8_t times, uint16_t delay_ms)
{
    // Implementation for blinking
    for(int i = 0; i < times; i++) {
        HAL_GPIO_WritePin(GPIO, GPIO_Pin, GPIO_PIN_SET);
        HAL_Delay(delay_ms);
        HAL_GPIO_WritePin(GPIO, GPIO_Pin, GPIO_PIN_RESET);
        HAL_Delay(delay_ms);
    }
}

void uxrce_app_on_topic( 
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
        (void) session; (void) object_id; (void) request_id; (void) stream_id; (void) length;

        AckermannDriveCmd topic;
        Uxrce_SubAckermann_OnTopic(ub, &topic);
}

void uxrce_app_init(void) {

    // 注册函数
    uxr_set_custom_transport_callbacks(&transport, true, uxrDds_UartOpen,
                                     uxrDds_UartClose, uxrDds_UartWrite,
                                     uxrDds_UartRead);
    // 初始化传输 Initialize transport
    uxr_init_custom_transport(&transport, &uart_args);
    transport.framing_io.local_addr = 0x01; // Example local address

    // 初始化会话 Session initialization
    uxr_init_session(&session, &transport.comm, 0xAAAABBBB);
    uxr_set_topic_callback(&session, uxrce_app_on_topic, NULL);

    for (int i = 0; i < 10; i++) {
        if (uxr_create_session(&session)) {
            session_ok = true;
            break;
        }
        printf("create session retry \r\n");
        HAL_Delay(300);
    }

    if (!session_ok) {
        printf("create session failed\r\n");
        return;
    }

    // 创建 reliable 流 Create reliable streams
    reliable_out =
        uxr_create_output_reliable_stream(&session,
                                        output_reliable_stream_buffer,
                                        BUFFER_SIZE,
                                        STREAM_HISTORY);
    reliable_in =
        uxr_create_input_reliable_stream(&session,
                                    input_reliable_stream_buffer,
                                    BUFFER_SIZE,
                                    STREAM_HISTORY);

    // 设置参与者 Participant 
    participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
            "<participant>"
            "<rtps>"
            "<name>default_xrce_participant</name>"
            "</rtps>"
            "</participant>"
            "</dds>";
    uint16_t participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0,
                    participant_xml, UXR_REPLACE);
    (void) participant_req;

    // Publish_HelloWorld_Init(&session, reliable_out, reliable_in, participant_id);
    Subscribe_Ackermann_Init(&session, reliable_out, reliable_in, participant_id);

    HAL_Delay(300);
}

void uxrce_app_loop(void) {
    
     if (!session_ok) {
        return;
    }
    // pub xrce
    // Publish_HelloWorld_Loop(&session, reliable_out);
    // sub ackermann
    uxr_run_session_time(&session, 3);
}
