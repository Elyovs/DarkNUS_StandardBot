/**
 * bsp_usart.h
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */

/** Instructions
 * 1) Configure USARTx and the corresponding DMA in CubeMX. Refer to instructions on GitHub.
 * 2) Include this header file.
 * 3) Start circular DMA process for detecting received data using usart_start(). Note
 *    that this function is blocking so this should be started in a freeRTOS task. Call
 *    this after all peripherals have been initialized.
 * 4) Define usart_ISR() somewhere in the code to respond to received data on USARTx.
 * 5) Whenever the DMA received data, the ISR will be triggered. To get the buffer of
 *    raw received data, use the usart_get_data() function.
 * 6) To sent data on the USARTx port, use the usart_send_data() function.
 */

#ifndef INC_BSP_USART_H_
#define INC_BSP_USART_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"


void init_xvr_usart(uint8_t *pData);
HAL_StatusTypeDef xvr_usart_start(UART_HandleTypeDef *huart,uint8_t *pData, uint16_t Size,queue_t *uart_queue);
void xvr_half_cplt_isr(DMA_HandleTypeDef *hdma);
void xvr_full_cplt_isr(DMA_HandleTypeDef *hdma);


HAL_StatusTypeDef ref_usart_start(UART_HandleTypeDef *huart,uint8_t *pData, uint16_t Size,queue_t *uart_queue);
void ref_half_cplt_isr(DMA_HandleTypeDef *hdma);
void ref_full_cplt_isr(DMA_HandleTypeDef *hdma);




#endif
