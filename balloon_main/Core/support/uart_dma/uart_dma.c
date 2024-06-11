/*
 * uart_dma.c
 *
 *  Created on: May 19, 2024
 *      Author: genta
 */

#include <string.h>
#include <stdio.h>
#include "uart_dma.h"



/**
 * \brief           USART DMA check thread
 * \param[in]       arg: Thread argument
 */
void
usart_rx_dma_thread(void* arg) {
    uart_desc_t* uart = arg;
    void* d;


    while (1) {
        /* Block thread and wait for event to process USART data */
        osMessageQueueGet(uart->data->queue, &d, NULL, osWaitForever);
        uint8_t a = 0;
        /* Simply call processing function */
        a = usart_rx_check(uart);
        printf("uart:%d\r\n", a);
        (void)d;
    }
}

/**
 * \brief           Check for new data received with DMA
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure
 * data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes,
 * hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
uint8_t
usart_rx_check(const uart_desc_t* uart) {
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(uart->data->dma_rx_buffer) - LL_DMA_GetDataLength(uart->dma_rx, uart->dma_rx_ch);
    if (pos != uart->data->old_pos) {           /* Check change in received data */
        if (pos > uart->data->old_pos) {        /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            return uart->data->dma_rx_buffer[uart->data->old_pos];
        } else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            return uart->data->dma_rx_buffer[uart->data->old_pos];
            if (pos > 0) {
                return uart->data->dma_rx_buffer[0];
            }
        }
        uart->data->old_pos = pos;              /* Save current position as old for next transfers */
    }
    return 0;

}

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */

/**
 * \brief           Send string to USART
 * \param[in]       str: String to send
 */
void
usart_send_string(const uart_desc_t* uart, const char* str) {
    usart_process_data(uart, str, strlen(str));
}

/**
 * \brief           General purpose DMA interrupt handler
 * \note            Function must be called from DMA interrupt
 *
 * It handles half-transfer and transfer-complete interrupts and does the job accordingly
 *
 * \param[in]       uart: Uart description to handle
 */
void
usart_dma_irq_handler(const uart_desc_t* uart) {
    void* d = (void *)1;

    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(uart->dma_rx, uart->dma_rx_ch) && uart->dma_rx_is_ht_fn(uart->dma_rx)) {
        uart->dma_rx_clear_ht_fn(uart->dma_rx); /* Clear half-transfer complete flag */
        osMessageQueuePut(uart->data->queue, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(uart->dma_rx, uart->dma_rx_ch) && uart->dma_rx_is_tc_fn(uart->dma_rx)) {
        uart->dma_rx_clear_tc_fn(uart->dma_rx); /* Clear transfer complete flag */
        osMessageQueuePut(uart->data->queue, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }
}

/**
 * \brief           General purpose UART interrupt handler
 * \note            Function must be called from UART interrupt
 *
 * It handles IDLE line detection interrupt and does the job accordingly
 *
 * \param[in]       uart: Uart description to handle
 */
void
usart_irq_handler(const uart_desc_t* uart) {
    void* d = (void *)1;

    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(uart->uart) && LL_USART_IsActiveFlag_IDLE(uart->uart)) {
        LL_USART_ClearFlag_IDLE(uart->uart);    /* Clear IDLE line flag */
        osMessageQueuePut(uart->data->queue, &d, 0, 0);  /* Write data to queue. Do not use wait function! */
    }
}
