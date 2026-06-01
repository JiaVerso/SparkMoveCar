#include "uxrce_app.h"
#include <uxr/client/client.h>

#include <ucdr/microcdr.h>
#include <string.h>

uxrCustomTransport transport;

void uxrce_ping_test_(void)
{

    uxr_set_custom_transport_callbacks(
    &transport,
    true,   // UART 是字节流，开 framing
    uart_open,
    uart_close,
    uart_write,
    uart_read);

uxr_init_custom_transport(&transport, NULL);

if (uxr_ping_agent_attempts(&transport.comm, 1000, 10)) {
    printf("XRCE Agent ping OK\r\n");
} else {
    printf("XRCE Agent ping FAIL\r\n");
}
}

