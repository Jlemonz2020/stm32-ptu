#include "uart_console.h"

#include <stdio.h>
#include <string.h>

#include "usart.h"

#define UART_CONSOLE_RX_LINE_LEN  (96U)
#define UART_CONSOLE_TX_BUF_LEN   (160U)

static uint8_t uart_rx_byte;
static char rx_line[UART_CONSOLE_RX_LINE_LEN];
static char rx_ready_line[UART_CONSOLE_RX_LINE_LEN];
static volatile uint16_t rx_index;
static volatile uint8_t rx_line_ready;

void uart_console_init(void)
{
  rx_index = 0U;
  rx_line_ready = 0U;
  (void)HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);
}

HAL_StatusTypeDef uart_console_write(const uint8_t *data, size_t len)
{
  if ((data == NULL) || (len == 0U)) {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(&huart3, (uint8_t *)data, (uint16_t)len, 10U);
}

int uart_console_printf(const char *fmt, ...)
{
  char buffer[UART_CONSOLE_TX_BUF_LEN];
  va_list args;
  int len;

  if (fmt == NULL) {
    return 0;
  }

  va_start(args, fmt);
  len = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (len <= 0) {
    return len;
  }

  if ((size_t)len > sizeof(buffer)) {
    len = (int)sizeof(buffer);
  }

  (void)uart_console_write((const uint8_t *)buffer, (size_t)len);
  return len;
}

void uart_console_poll_rx(void)
{
}

void uart_console_on_rx_byte(uint8_t byte)
{
  if ((byte == '\r') || (byte == '\n')) {
    if (rx_index > 0U) {
      rx_line[rx_index] = '\0';
      (void)strncpy(rx_ready_line, rx_line, sizeof(rx_ready_line));
      rx_ready_line[sizeof(rx_ready_line) - 1U] = '\0';
      rx_line_ready = 1U;
      rx_index = 0U;
    }
    return;
  }

  if (rx_index < (UART_CONSOLE_RX_LINE_LEN - 1U)) {
    rx_line[rx_index] = (char)byte;
    rx_index++;
  } else {
    rx_index = 0U;
  }
}

uint8_t uart_console_get_line(char *line, size_t line_len)
{
  if ((line == NULL) || (line_len == 0U) || (rx_line_ready == 0U)) {
    return 0U;
  }

  __disable_irq();
  (void)strncpy(line, rx_ready_line, line_len);
  line[line_len - 1U] = '\0';
  rx_line_ready = 0U;
  __enable_irq();

  return 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) {
    uart_console_on_rx_byte(uart_rx_byte);
    (void)HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);
  }
}
