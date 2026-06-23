#ifndef UART_CONSOLE_H
#define UART_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"

void uart_console_init(void);
HAL_StatusTypeDef uart_console_write(const uint8_t *data, size_t len);
int uart_console_printf(const char *fmt, ...);
void uart_console_poll_rx(void);
void uart_console_on_rx_byte(uint8_t byte);
uint8_t uart_console_get_line(char *line, size_t line_len);

#ifdef __cplusplus
}
#endif

#endif
