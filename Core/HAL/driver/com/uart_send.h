#ifndef _H_UART_SEND_H
#define _H_UART_SEND_H
#include "datatypes.h"

#pragma pack(1)
typedef struct uart_send_config_t{
    uint8_t bsp_uart_index;
    uint8_t data_len;
} uart_send_config;

typedef struct uart_send_t{
    uart_send_config config;
    uint8_t* txbuf;
    uint8_t buf_len;
} uart_send;
#pragma pack(1)

uart_send* UartSend_Create(uart_send_config* config);
void UartSend_Send(uart_send* obj,uint8_t* data);

#endif