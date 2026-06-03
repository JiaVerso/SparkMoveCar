#include "uxrce_app.h"
#include "usart.h"
#include <uxr/client/client.h>
#include "gpio.h"

#include <string.h>
#include <ucdr/microcdr.h>
#include <uxr/client/util/ping.h>
#include <stdio.h>

#include "uxrce_pub_topics.h"

uxrCustomTransport transport;

// uart_args.huart = &huart8
uxrUartTransArgs_t uart_args = {
    .huart = &huart8,
};

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

void uxrce_app_init(void) {

    // 注册回调 Register callback
    uxr_set_custom_transport_callbacks(&transport, true, uxrDds_UartOpen,
                                     uxrDds_UartClose, uxrDds_UartWrite,
                                     uxrDds_UartRead);

    uxr_init_custom_transport(&transport, &uart_args);

    transport.framing_io.local_addr = 0x01; // Example local address

    Publish_HelloWorld_Init(&transport, 0, NULL);
}


void uxrce_app_loop(void) {
    // write topics
    Publish_HelloWorld_Loop();
}
