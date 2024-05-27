/*
 * uart_dma.h
 *
 *  Created on: May 19, 2024
 *      Author: genta
 */

#ifndef SUPPORT_UART_DMA_UART_DMA_H_
#define SUPPORT_UART_DMA_UART_DMA_H_
#include <stdint.h>
#include "stm32l4xx_hal.h"

#include "cmsis_os.h"
#include "fatfs.h"

#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_dma.h"

#include "stm32l4xx_ll_exti.h"

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

typedef struct {
    /* OS queue */
    osMessageQueueId_t queue;                   /*!< Message queue */

    /* Raw data buffer */
    uint8_t dma_rx_buffer[64];                  /*!< DMA buffer for receive data */
    size_t old_pos;                             /*!< Position for data */
} uart_desc_volatile_t;

/**
 * \brief           Non-Volatile data structure
 *
 * Variables declared using this structure may be put to non-volative memory.
 * This is to avoid using RAM for constant data
 */
typedef struct {
    /* UART config */
    USART_TypeDef* uart;                        /*!< UART/USART/LPUART instance */

    /* DMA config & flags management */
    DMA_TypeDef* dma_rx;                        /*!< RX DMA instance */
    uint32_t dma_rx_ch;                         /*!< RX DMA channel */
    void (*dma_rx_clear_tc_fn)(DMA_TypeDef *);
    void (*dma_rx_clear_ht_fn)(DMA_TypeDef *);
    uint32_t (*dma_rx_is_tc_fn)(DMA_TypeDef *);
    uint32_t (*dma_rx_is_ht_fn)(DMA_TypeDef *);

    /* Interrupts config */
    uint8_t prio;                               /*!< Preemption priority number */
    IRQn_Type uart_irq;                         /*!< UART IRQ instance */
    IRQn_Type dma_irq;                          /*!< DMA IRQ instance */

    uart_desc_volatile_t* data;                 /*!< Pointer to volatile data */
} uart_desc_t;



void usart_rx_check(const uart_desc_t* uart);
void usart_process_data(const uart_desc_t* uart, const void* data, size_t len);
void usart_send_string(const uart_desc_t* uart, const char* str);
void usart_dma_irq_handler(const uart_desc_t* uart);
void usart_irq_handler(const uart_desc_t* uart);

void usart_rx_dma_thread(void* arg);

#define DECLARE_UART_DMA_DESC(UART_N, DMA_N, DMA_CHANNEL_N) \
uart_desc_volatile_t uart##UART_N##_desc_data; \
const uart_desc_t uart##UART_N##_desc = \
         { \
        .uart = USART ## UART_N , \
        /* DMA config */ \
        .dma_rx = DMA##DMA_N, \
        .dma_rx_ch = LL_DMA_CHANNEL_##DMA_CHANNEL_N,   \
        .dma_irq = DMA1_Channel##DMA_CHANNEL_N##_IRQn, \
        .uart_irq = USART##UART_N##_IRQn, \
        .prio = 5, \
        .dma_rx_clear_tc_fn = LL_DMA_ClearFlag_TC##DMA_CHANNEL_N, \
        .dma_rx_clear_ht_fn = LL_DMA_ClearFlag_HT##DMA_CHANNEL_N, \
        .dma_rx_is_tc_fn = LL_DMA_IsActiveFlag_TC##DMA_CHANNEL_N, \
        .dma_rx_is_ht_fn = LL_DMA_IsActiveFlag_HT##DMA_CHANNEL_N, \
        /* Volatile data */ \
        .data = &uart##UART_N##_desc_data, \
};

#define INIT_UART_DMA_DESC(UART_N, DMA_N, DMA_CHANNEL_N) \
LL_DMA_EnableIT_HT(DMA##DMA_N, LL_DMA_CHANNEL_##DMA_CHANNEL_N); \
LL_DMA_EnableIT_TC(DMA##DMA_N, LL_DMA_CHANNEL_##DMA_CHANNEL_N); \
uart##UART_N##_desc.data->queue = osMessageQueueNew(10, sizeof(void *), NULL);
#define START_UART_DMA(UART_N) \
osThreadNew(usart_rx_dma_thread, (void *)&uart##UART_N##_desc, NULL);
#endif /* SUPPORT_UART_DMA_UART_DMA_H_ */
