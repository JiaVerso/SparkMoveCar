#include "uxrce_app.h"
#include "usart.h"
#include <uxr/client/client.h>
#include "gpio.h"

#include <string.h>
#include <ucdr/microcdr.h>
#include <uxr/client/util/ping.h>
#include <stdio.h>

uxrCustomTransport transport;

// uart_args.huart = &huart8
uxrUartTransArgs_t uart_args = {
    .huart = &huart8,
};

static void uxrce_blink(GPIO_TypeDef* GPIO, uint16_t GPIO_Pin, uint8_t times, uint16_t delay_ms)
{
    // Implementation for blinking
    for(int i = 0; i < times; i++) {
        HAL_GPIO_WritePin(GPIO, GPIO_Pin, GPIO_PIN_SET);
        HAL_Delay(delay_ms);
        HAL_GPIO_WritePin(GPIO, GPIO_Pin, GPIO_PIN_RESET);
        HAL_Delay(delay_ms);
    }
}

void uxrce_ping_test_(void) {

    // 注册回调 Register callback
  uxr_set_custom_transport_callbacks(&transport, true, uxrDds_UartOpen,
                                     uxrDds_UartClose, uxrDds_UartWrite,
                                     uxrDds_UartRead);

  uxr_init_custom_transport(&transport, &uart_args);

  if (uxr_ping_agent_attempts(&transport.comm, 1000, 10)) {
    // 绿灯闪烁 Green LED blinks
    uxrce_blink(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 5, 100);
  } else {
    uxrce_blink(LED_RED_GPIO_Port, LED_RED_Pin, 5, 100);
  }
}
