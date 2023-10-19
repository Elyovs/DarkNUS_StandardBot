/*
 * bsp_dbus_input.h
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */

/** Instructions
 * 1) Configure USART1 and DMA2 in CubeMX. Refer to instructions on GitHub.
 * 2) Include this header file.
 * 3) Start circular DMA process using dbus_remote_start(). Note that this function
 *    is blocking so this should be started in a freeRTOS task. Call this after
 *    all peripherals have been initialized.
 * 4) Define dbus_remote_ISR() somewhere in the code to respond to commands sent
 *    by the controller.
 * 5) Whenever the DMA receives data, the ISR will be triggered. When triggered, data can
 *    be read through the remote_cmd struct, like get_dbus_remote_cmd()->right_y.
 */

#ifndef INC_BSP_DBUS_INPUT_H_
#define INC_BSP_DBUS_INPUT_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"

/* Size in bytes of data sent from remote */

/* Pointer to the received raw data from the remote */
extern uint8_t remote_raw_data[REMOTE_DATA_SIZE];

/**
 * Internal callback function that also converts raw data from the controller to cleaned
 * data that can be used by the system. This function in turn called the ISR,
 * dbus_remote_ISR()
 */
void dbus_remote_ISR(DMA_HandleTypeDef *hdma);

/**
 * This function starts the circular DMA that reads from the USART1 port to memory.
 * This function is blocking, so it is advised to only call this inside a freeRTOS
 * task so that execution flow can continue thanks to preemption.
 *
 * @param huart  Pointer to the UART port handle. This should be the DBUS UART1 port.
 * @param pData  Pointer to the buffer where the received data will be stored. Ideally
 *               this should be the remote_raw_data buffer defined in this header file.
 */
HAL_StatusTypeDef dbus_remote_start();

#endif
