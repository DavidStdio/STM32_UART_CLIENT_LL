/*
 * uart_wrapper.c
 *
 *  Created on: Feb 10, 2020
 *      Author: E014283
 */
#include <stdint.h>
#include "main.h"
//#include "stm32f7xx_hal.h"
//#include "stm32f7xx_hal_conf.h"
//#include "stm32f7xx_ll_rcc.h"
//#include "stm32f7xx_ll_bus.h"
//#include "stm32f7xx_ll_system.h"
//#include "stm32f7xx_ll_exti.h"
//#include "stm32f7xx_ll_cortex.h"
//#include "stm32f7xx_ll_utils.h"
//#include "stm32f7xx_ll_pwr.h"
//#include "stm32f7xx_ll_dma.h"
//#include "stm32f7xx_ll_usart.h"
//#include "stm32f7xx.h"
//#include "stm32f7xx_ll_gpio.h"

static size_t usart_tx_dma_current_len = 0;

__weak void uart_rxcpltcallback(USART_TypeDef* usart)
{
  UNUSED(usart);
}

__weak void uart_txcpltcallback(USART_TypeDef* usart)
{
  UNUSED(usart);
}

__weak void uart_abrtcpltcallback(USART_TypeDef* usart)
{
  UNUSED(usart);
}

/**
 * \brief           DMA1 stream1 interrupt handler for USART3 RX
 */
void DMA1_Stream1_IRQHandler(void)
{
    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_TC1(DMA1);             /* Clear transfer complete flag */
        uart_rxcpltcallback(USART3);                       /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
 * \brief           DMA1 stream1 interrupt handler for USART3 TX
 */
void DMA1_Stream3_IRQHandler(void)
{
    /* Check transfer complete */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_3) && LL_DMA_IsActiveFlag_TC3(DMA1)) {
        LL_DMA_ClearFlag_TC3(DMA1);             /* Clear transfer complete flag */
        usart_tx_dma_current_len = 0;           /* Clear length variable */
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
		uart_txcpltcallback(USART3);
    }
}

/**
 * \brief           USART3 global interrupt handler
 */
void USART3_IRQHandler(void)
{
	/* Check for Character Match interrupt */
	if (LL_USART_IsEnabled(USART3) && LL_USART_IsActiveFlag_CM(USART3))
	{
        LL_USART_ClearFlag_CM(USART3);        /* Clear Character Match flag */
		if ( LL_USART_IsEnabledDMAReq_RX(USART3) ) //HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)
		{
			//CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
			LL_USART_DisableDMAReq_RX(USART3);
			LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
		}
        uart_abrtcpltcallback(USART3);          /* Check for data to process */
	}

    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART3) && LL_USART_IsActiveFlag_IDLE(USART3))
    {
        LL_USART_ClearFlag_IDLE(USART3);        /* Clear IDLE line flag */
        uart_rxcpltcallback(USART3);         	/* Check for data to process */
    }

    /* Implement other events when needed */
}


uint8_t usart_start_tx_dma_transfer(uint8_t *pBuf, uint32_t len)
{
    uint32_t old_primask;
    uint8_t started = 0;

    /* Check if DMA is active */
    /* Must be set to 0 */
    old_primask = __get_PRIMASK();
    __disable_irq();

    /* Check if transfer is not active */
    if (usart_tx_dma_current_len == 0) {
        /* Check if something to send  */
        usart_tx_dma_current_len = len;
            /* Disable channel if enabled */
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);

            /* Clear all flags */
		LL_DMA_ClearFlag_TC3(DMA1);
		LL_DMA_ClearFlag_HT3(DMA1);
		LL_DMA_ClearFlag_TE3(DMA1);
		LL_DMA_ClearFlag_DME3(DMA1);
		LL_DMA_ClearFlag_FE3(DMA1);

            /* Start DMA transfer */
		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, len);
		LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)pBuf);
	    LL_DMA_SetPeriphAddress(DMA1,
								LL_DMA_STREAM_3,
								LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT));

            /* Start new transfer */
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
		LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);
		LL_USART_EnableDMAReq_TX(USART3);
		started = 1;
    }

    __set_PRIMASK(old_primask);
    return started;
}


uint8_t usart_start_rx_dma_transfer(uint8_t *pBuf, uint32_t len)
{
	LL_DMA_SetPeriphAddress(DMA1,
			LL_DMA_STREAM_1,
			LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)pBuf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, len);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);

	LL_USART_EnableIT_CM(USART3);
    LL_USART_EnableDMAReq_RX(USART3);
	return 1;
}
