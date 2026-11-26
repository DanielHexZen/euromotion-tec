/* com_uart.c — spec §7.22 (REQ-COM-02..06). Main loop owns the TX buffers; the ISR only clears
 * `busy` and pushes RX bytes, so no buffer is shared between contexts.
 * USART2: RX by 1-byte interrupt into an SPSC ring, TX by interrupt.
 * USART1: RX by circular DMA into the ring, read up to the DMA write index (NDTR) in every
 * com_uart_poll (1 kHz); TX by DMA. No IDLE-line interrupt: the 1 kHz poll already sees every
 * byte within 1 ms, and 256 B last 133 ms at 19200 Bd (D-80). */
#include "com_uart.h"

#include <string.h>

#include "console.h"
#include "sys_time.h"
#include "tlm.h"

#define RX_N 256U            /* power of two */

typedef struct {
    UART_HandleTypeDef *h;
    bool     dma;                               /* USART1: RX circular DMA, TX DMA          */
    uint8_t  rx_byte;
    uint8_t  rx[RX_N];
    volatile uint32_t head, tail;               /* IT port: head ISR, tail main (REQ-EXE-06 c) */
    volatile bool rx_restart;                   /* DMA port: re-armed by the error ISR      */
    uint8_t *tx[2];
    uint16_t tx_size;
    uint8_t  fill;                              /* buffer index being filled              */
    uint16_t fill_len;
    volatile bool busy;                         /* set by main before starting, cleared by ISR */
    com_mode_t mode;
    com_stats_t st;
} port_t;

static uint8_t tx0[2][256], tx1[2][128];
static port_t P[COM_N] = {
    [COM_USB]   = { .tx = { tx0[0], tx0[1] }, .tx_size = 256U },
    [COM_RADIO] = { .tx = { tx1[0], tx1[1] }, .tx_size = 128U, .dma = true },
};

static port_t *by_handle(UART_HandleTypeDef *h)
{
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) {
        if (P[i].h == h) { return &P[i]; }
    }
    return NULL;
}

static void rx_arm(port_t *p)
{
    if (p->dma) { (void)HAL_UART_Receive_DMA(p->h, p->rx, RX_N); }
    else        { (void)HAL_UART_Receive_IT(p->h, &p->rx_byte, 1U); }
}

void com_uart_init(UART_HandleTypeDef *h2, UART_HandleTypeDef *h1)
{
    P[COM_USB].h = h2;
    P[COM_RADIO].h = h1;
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) { rx_arm(&P[i]); }
}

static void tx_kick(port_t *p)
{
    if (p->busy || p->fill_len == 0U) { return; }
    const uint8_t *buf = p->tx[p->fill];
    const uint16_t len = p->fill_len;
    p->fill ^= 1U;
    p->fill_len = 0U;
    p->busy = true;
    const HAL_StatusTypeDef s = p->dma ? HAL_UART_Transmit_DMA(p->h, buf, len) : HAL_UART_Transmit_IT(p->h, buf, len);
    if (s != HAL_OK) {                                          /* cannot happen while !busy */
        p->busy = false;
        p->st.errors++;
    }
}

static void deliver(com_port_t i, uint8_t b)
{
    if (P[i].mode == COM_BINARY) { tlm_rx_byte((unsigned)i, b); }
    else { con_rx_byte(i, b); }
}

void com_uart_poll(void)
{
    for (uint32_t i = 0U; i < (uint32_t)COM_N; i++) {
        port_t *p = &P[i];
        if (p->dma) {
            if (p->rx_restart) { p->rx_restart = false; p->tail = 0U; }
            /* NDTR counts RX_N .. 1 and reloads to RX_N, so the write index is RX_N - NDTR mod RX_N */
            const uint32_t head = (RX_N - (uint32_t)__HAL_DMA_GET_COUNTER(p->h->hdmarx)) & (RX_N - 1U);
            while (p->tail != head) {
                deliver((com_port_t)i, p->rx[p->tail]);
                p->st.rx_bytes++;
                p->tail = (p->tail + 1U) & (RX_N - 1U);
            }
        } else {
            while (p->tail != p->head) {
                deliver((com_port_t)i, p->rx[p->tail & (RX_N - 1U)]);
                p->tail++;
            }
        }
        tx_kick(p);
    }
}

size_t com_write(com_port_t pi, const uint8_t *d, size_t n)
{
    port_t *p = &P[pi];
    if (n == 0U) { return 0U; }
    if (n > (size_t)(p->tx_size - p->fill_len)) { p->st.tx_drops++; return 0U; }
    memcpy(&p->tx[p->fill][p->fill_len], d, n);
    p->fill_len = (uint16_t)(p->fill_len + n);
    tx_kick(p);
    return n;
}

size_t com_tx_free(com_port_t pi) { return (size_t)(P[pi].tx_size - P[pi].fill_len); }
bool com_tx_idle(com_port_t pi) { return !P[pi].busy && P[pi].fill_len == 0U; }
com_stats_t com_stats(com_port_t pi) { return P[pi].st; }
com_mode_t com_mode(com_port_t pi) { return P[pi].mode; }
void com_set_mode(com_port_t pi, com_mode_t m) { P[pi].mode = m; }

void com_flush(uint32_t timeout_ms)
{
    const uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms && !(com_tx_idle(COM_USB) && com_tx_idle(COM_RADIO))) {
        com_uart_poll();
    }
}

/* The DMA port also lands here (circular transfer complete): nothing to do, poll reads NDTR. */
void com_rx_cplt_isr(UART_HandleTypeDef *h)
{
    port_t *p = by_handle(h);
    if (p == NULL || p->dma) { return; }
    if (p->head - p->tail < RX_N) {
        p->rx[p->head & (RX_N - 1U)] = p->rx_byte;
        p->head++;
        p->st.rx_bytes++;
    } else {
        p->st.rx_overruns++;
    }
    (void)HAL_UART_Receive_IT(h, &p->rx_byte, 1U);
}

void com_tx_cplt_isr(UART_HandleTypeDef *h)
{
    port_t *p = by_handle(h);
    if (p != NULL) { p->busy = false; }
}

/* REQ-COM-05: reception must never stay disarmed. On the IT port the HAL may have kept the
 * receiver armed (HAL_BUSY, harmless). With DMA reception every error aborts the transfer
 * (DMAR set), so RxState is READY here and the DMA restarts at index 0. */
void com_error_isr(UART_HandleTypeDef *h)
{
    port_t *p = by_handle(h);
    if (p == NULL) { return; }
    p->st.errors++;
    if (h->gState == HAL_UART_STATE_READY) { p->busy = false; }   /* a TX aborted by the error */
    if (p->dma) {
        if (h->RxState == HAL_UART_STATE_READY) { p->rx_restart = true; rx_arm(p); }
    } else {
        rx_arm(p);
    }
}
