/* com_uart.h — serial ports (spec §7.22). Device layer. Phase 0: both ports in interrupt
 * mode, text only; USART1 DMA and the BINARY mode arrive with phase 4. */
#ifndef COM_UART_H
#define COM_UART_H

#include <stddef.h>

#include "main.h"
#include "units.h"

typedef enum { COM_USB = 0, COM_RADIO = 1, COM_N } com_port_t;

typedef struct {
    uint32_t rx_bytes, rx_overruns, tx_drops, errors;
} com_stats_t;

/* Init only. Arms both receivers. */
void   com_uart_init(UART_HandleTypeDef *h2, UART_HandleTypeDef *h1);
/* S_FAST: hands received bytes to the console and starts the next TX buffer if idle. */
void   com_uart_poll(void);
/* Main loop, non-blocking. Drops the whole write if it does not fit (REQ-COM-03). */
size_t com_write(com_port_t p, const uint8_t *d, size_t n);
size_t com_tx_free(com_port_t p);              /* bytes the filling buffer can still take */
bool   com_tx_idle(com_port_t p);
/* Init only: wait (bounded) until both TX paths are empty. */
void   com_flush(uint32_t timeout_ms);
com_stats_t com_stats(com_port_t p);

/* ISR entry points, called from app_isr.c. */
void   com_rx_cplt_isr(UART_HandleTypeDef *h);
void   com_tx_cplt_isr(UART_HandleTypeDef *h);
void   com_error_isr(UART_HandleTypeDef *h);

#endif
