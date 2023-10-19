/**
 * bsp_can.h
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */

/** Instructions
 * 1) Configure CAN1 and CAN2 busses in CubeMX. Refer to instructions on GitHub.
 * 2) Include this header file.
 * 3) Start CAN process using can_start(). This starts a singular CAN
 *    bus like CAN1 or CAN2. You will have to call it twice to enable both buses.
 * 4) Define can_ISR() somewhere in the code so it triggers when a message arrives
 *    at the CAN1 or CAN2 mailboxes.
 * 5) can_get_msg() can be called inside the can_ISR() to get the latest message.
 * 6) Messages can be sent via the can_send_msg() function.
 */

#ifndef INC_BSP_CAN_H_
#define INC_BSP_CAN_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"

#define CAN_BUFFER_SIZE 8

void can_ISR(CAN_HandleTypeDef *hcan);

/**
 * Reads the latest message that has arrived on the specified CAN bus and stores
 * it in a buffer. This function is ideally called inside the can_ISR() function
 * once it is defined somewhere in the code.
 *
 * @param hcan          A pointer to the CAN1 or CAN2 bus handle to use.
 * @param rx_msg_header Struct that contains data on which device the incoming message
 *                      came from.
 * @param rx_buffer     Buffer to store received data in. Minimum size of this buffer
 *                      must be 8 bytes.
 */
HAL_StatusTypeDef can1_get_msg(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_msg_header, uint8_t *rx_buffer);
HAL_StatusTypeDef can2_get_msg(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_msg_header, uint8_t *rx_buffer);
/**
 * Sends a message on the selected CAN bus.
 *
 * @param hcan           A pointer to the CAN1 or CAN2 bus handle to use.
 * @param destination_id A hexadecimal identifier for the device the CAN message
 *                       must be sent to.
 * @param tx_buffer      Buffer of size buffer_size containing data to send to
 *                       the specified device.
 * @param buffer_size    Size in bytes of the tx_buffer.
 */
HAL_StatusTypeDef can_send_msg(CAN_HandleTypeDef *hcan, uint32_t destination_id,
		uint8_t *tx_buffer, uint16_t buffer_size);


/**
 * @brief  initialises CAN filter, then starts the CAN interrupts
 * for CAN1, messages go into FIFO0. For CAN2, messages go into FIFO1
 *
 * @param *hcan pointer to the CANbus being initialised
 * @param CAN_filterID 32bit CAN ID filter
 * @param CAN_filtermask 32bit CAN ID mask
 * @usage Call during initialisation to setup filters, start CAN and start ISRs
 */
void can_start(CAN_HandleTypeDef *hcan, uint32_t CAN_filterID, uint32_t CAN_filterMask);

#endif
