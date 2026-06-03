#include <stdint.h>
#include <stdbool.h>

#include "uxrce_app.h"
#include "stm32f4xx_hal.h"

#include "uxrce_transport_uart.h"

#define UXRCE_UART_RX_RING_SIZE 1024U

extern uxrCustomTransport transport;

#define UXRCE_UART_RX_RING_SIZE 1024U

static uint8_t rx_ring[UXRCE_UART_RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static uint16_t ring_next(uint16_t value)
{
    return (uint16_t)((value + 1U) % UXRCE_UART_RX_RING_SIZE);
}

void uxrDds_UartRxCallback(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = ring_next(rx_head);

        if (next == rx_tail) {
            break;
        }

        rx_ring[rx_head] = data[i];
        rx_head = next;
    }
}
/*
* -----------------------------------------------------------------------------
*/
bool uxrDds_UartOpen(uxrCustomTransport *transport)
{
    uxrUartTransArgs_t *args =
        (uxrUartTransArgs_t *)transport->args;

    if (args == NULL || args->huart == NULL) {
        return false;
    }

    rx_head = 0;
    rx_tail = 0;

    return true;
}

/*
* -----------------------------------------------------------------------------
*/
bool uxrDds_UartClose(uxrCustomTransport *transport)
{
    (void)transport;
    return true;
}

/*
* -----------------------------------------------------------------------------
*/
size_t uxrDds_UartWrite(
    uxrCustomTransport *transport,
    const uint8_t* buffer,
    size_t length,
    uint8_t *error_code)
{
    uxrUartTransArgs_t *args =
        (uxrUartTransArgs_t *)transport->args;

    if (args == NULL || args->huart == NULL || buffer == NULL) {
        *error_code = 1;
        return 0;
    }

    HAL_StatusTypeDef ret = HAL_UART_Transmit(
        args->huart,
        (uint8_t *)buffer,
        (uint16_t)length,
                100);

    if (ret == HAL_OK) {
        *error_code = 0;
        return length;
    }

    *error_code = 2;
    return 0;
}

/*
* -----------------------------------------------------------------------------
*/
size_t uxrDds_UartRead(uxrCustomTransport *transport,
                       uint8_t *buffer,
                       size_t length,
                       int timeout,
                       uint8_t *error_code)
{
    (void)transport;

    uint32_t start = HAL_GetTick();
    size_t count = 0;

    while (count < length) {
        if (rx_tail != rx_head) {
            buffer[count++] = rx_ring[rx_tail];
            rx_tail = ring_next(rx_tail);
            continue;
        }

        if ((int)(HAL_GetTick() - start) >= timeout) {
            break;
        }
    }

    if (error_code != NULL) {
        *error_code = 0;
    }

    return count;
}