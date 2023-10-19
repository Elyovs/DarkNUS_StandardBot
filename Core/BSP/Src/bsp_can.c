/**
 * bsp_can.c
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */

#include "bsp_can.h"

/** Note
 * Currently only the CAN FIFO0 has been implemented. An implementation of FIFO1
 * simultaneously wouldn't be too hard to do.
 */

/**
 * HAL internal callback function that calls abstracted ISR for ease of use.
 * Define can_ISR() elsewhere in code to define behaviour of CAN receive ISR.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	can_ISR(hcan);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	can_ISR(hcan);
}

HAL_StatusTypeDef can1_get_msg(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_msg_header, uint8_t *rx_buffer)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, rx_msg_header, rx_buffer);
	return HAL_OK;
}

HAL_StatusTypeDef can2_get_msg(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_msg_header, uint8_t *rx_buffer)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, rx_msg_header, rx_buffer);
	return HAL_OK;
}

/**
 * @brief  initialises CAN filter, then starts the CAN interrupts
 * for CAN1, messages go into FIFO0. For CAN2, messages go into FIFO1
 *
 * @param *hcan pointer to the CANbus being initialised
 * @param CAN_filterID 32bit CAN ID filter
 * @param CAN_filtermask 32bit CAN ID mask
 * @usage Call during initialisation to setup filters, start CAN and start ISRs
 */
void can_start(CAN_HandleTypeDef *hcan, uint32_t CAN_filterID, uint32_t CAN_filterMask) {
    CAN_FilterTypeDef can_filter_st = {0};
    can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = (CAN_filterID >> 16);
	can_filter_st.FilterIdLow = (CAN_filterID & 0xFFFF);
	can_filter_st.FilterMaskIdHigh = (CAN_filterMask >> 16);
	can_filter_st.FilterMaskIdLow = (CAN_filterMask & 0xFFFF);

	if (hcan->Instance == CAN1) {
	    can_filter_st.FilterBank = 0;
	    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	    HAL_CAN_ConfigFilter(hcan, &can_filter_st);
	    HAL_CAN_Start(hcan);
	    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	} else if (hcan->Instance == CAN2) {
	    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1; // Uncomment line if using RX1 queue as well.
		can_filter_st.SlaveStartFilterBank = 14;
		can_filter_st.FilterBank = 14;
	    HAL_CAN_ConfigFilter(hcan, &can_filter_st);
	    HAL_CAN_Start(hcan);
	    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // Uncomment line if using RX1 queue as well.
	}

}
