#ifndef __UART_WRAPPER
#define __UART_WRAPPER

uint8_t usart_start_tx_dma_transfer(uint8_t *pBuf, uint32_t len);
uint8_t usart_start_rx_dma_transfer(uint8_t *pBuf, uint32_t len);

#endif
