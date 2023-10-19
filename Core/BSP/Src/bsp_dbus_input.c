/*
 * bsp_dbus_input.c
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */
#include "board_lib.h"
#include "robot_config.h"
#include "bsp_dbus_input.h"

#define JOYSTICK_OFFSET 1024
extern TaskHandle_t control_input_task_handle;

uint8_t remote_raw_data[REMOTE_DATA_SIZE] = {0};
remote_cmd_t remote_cmd = { 0 };


void dbus_remote_ISR(DMA_HandleTypeDef *hdma) {
	remote_cmd.right_x = (remote_raw_data[0] | remote_raw_data[1] << 8) & 0x07FF;
	remote_cmd.right_x -= JOYSTICK_OFFSET;
	remote_cmd.right_y = (remote_raw_data[1] >> 3 | remote_raw_data[2] << 5) & 0x07FF;
	remote_cmd.right_y -= JOYSTICK_OFFSET;
	remote_cmd.left_x = (remote_raw_data[2] >> 6 | remote_raw_data[3] << 2
			| remote_raw_data[4] << 10) & 0x07FF;
	remote_cmd.left_x -= JOYSTICK_OFFSET;
	remote_cmd.left_y = (remote_raw_data[4] >> 1 | remote_raw_data[5] << 7) & 0x07FF;
	remote_cmd.left_y -= JOYSTICK_OFFSET;
	//Left switch position
	remote_cmd.left_switch = ((remote_raw_data[5] >> 4) & 0x000C) >> 2;
	remote_cmd.right_switch = (remote_raw_data[5] >> 4) & 0x0003;
	remote_cmd.mouse_x = ((int16_t) remote_raw_data[6] | ((int16_t) remote_raw_data[7] << 8));
	remote_cmd.mouse_y = ((int16_t) remote_raw_data[8] | ((int16_t) remote_raw_data[9] << 8));
	remote_cmd.mouse_z = ((int16_t) remote_raw_data[10] | ((int16_t) remote_raw_data[11] << 8));
	remote_cmd.mouse_left = (remote_raw_data[12]);
	remote_cmd.mouse_right = (remote_raw_data[13]);
	remote_cmd.keyboard_keys = (remote_raw_data[14]);
	remote_cmd.side_dial = ((int16_t) remote_raw_data[16]) | ((int16_t) remote_raw_data[17] << 8);
	remote_cmd.side_dial -= JOYSTICK_OFFSET;
	remote_cmd.last_time = HAL_GetTick();

	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(control_input_task_handle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * This function starts the circular DMA for receiving on a UART port. It is specifically
 * written for the UART1 port for DBUS interface from the controller.
 */
HAL_StatusTypeDef dbus_remote_start()
{
	uint8_t *pData = remote_raw_data;
	UART_HandleTypeDef *huart = &DBUS_UART;
	uint32_t *tmp;

	/* Check that a Rx process is not already ongoing */
	if (huart->RxState == HAL_UART_STATE_READY) {
		if ((pData == NULL) || (REMOTE_DATA_SIZE == 0U)) {
			return HAL_ERROR;
		}

		/* Process Locked */
		__HAL_LOCK(huart);

		huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = REMOTE_DATA_SIZE;

		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_BUSY_RX;

		/* Set the UART DMA transfer complete callback */
		huart->hdmarx->XferCpltCallback = dbus_remote_ISR;

		/* Set the DMA abort callback */
		huart->hdmarx->XferAbortCallback = NULL;

		/* Enable the DMA stream */
		tmp = (uint32_t *)&pData;
		HAL_DMA_Start_IT(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t *)tmp, REMOTE_DATA_SIZE);

		/* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
		__HAL_UART_CLEAR_OREFLAG(huart);

		/* Process Unlocked */
		__HAL_UNLOCK(huart);

		/* Enable the UART Parity Error Interrupt */
		SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);

		/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
		in the UART CR3 register */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
		{
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
		}
		else
		{
			/* In case of errors already pending when reception is started,
			   Interrupts may have already been raised and lead to reception abortion.
			   (Overrun error for instance).
			   In such case Reception Type has been reset to HAL_UART_RECEPTION_STANDARD. */
			return HAL_ERROR;
		}
		return HAL_OK;
	} else {
		return HAL_BUSY;
	}
}
