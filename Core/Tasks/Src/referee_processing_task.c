/*
 * referee_processing_task.c
 *
 *  Created on: Jun 18, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "bsp_queue.h"
#include "bsp_referee.h"
#include "bsp_usart.h"
#include "referee_processing_task.h"
#include "referee_msgs.h"
#include "robot_config.h"
#include "rtos_g_vars.h"

extern int g_spinspin_mode;
extern uint8_t remote_raw_data[18];
extern TaskHandle_t referee_processing_task_handle;
referee_limit_t referee_limiters;
ref_msg_t msg_buffer;
ref_game_state_t ref_game_state;
uint32_t ref_game_state_txno = 0;
ref_game_robot_HP_t ref_robot_hp;
uint32_t ref_robot_hp_txno = 0;
ref_game_robot_data_t ref_robot_data;
uint32_t ref_robot_data_txno = 0;
ref_robot_power_data_t ref_power_data;
uint32_t ref_power_data_txno = 0;

ref_game_robot_pos_t ref_robot_pos;
uint32_t ref_robot_pos_txno = 0;

ref_buff_data_t ref_buff_data;
uint32_t ref_buff_data_txno = 0;

ref_robot_dmg_t ref_dmg_data;
uint32_t ref_dmg_data_txno = 0;

ref_shoot_data_t ref_shoot_data;
uint32_t ref_shoot_data_txno = 0;

ref_magazine_data_t ref_mag_data;
uint32_t ref_mag_data_txno = 0;
uint8_t g_ref_tx_seq = 0;


void referee_processing_task(void *argument) {
	referee_limiters.robot_level = 0;
	queue_t referee_uart_q;
	uint8_t ref_buffer[2];
	ref_processing_status_t proc_status;
	referee_limiters.feeding_speed = LV1_FEEDER;
	referee_limiters.projectile_speed = LV1_PROJECTILE;
	referee_limiters.wheel_power_limit = LV1_POWER * CHASSIS_POWER_MULT;
	referee_limiters.robot_level = 1;
	status_led(7, on_led);
	status_led(8, off_led);
	uint8_t graphic_sent = 0;
	ref_robot_data.robot_id = 0;
	ref_usart_start(&REFEREE_UART, ref_buffer, 2, &referee_uart_q);
	while (1) {
//			uint32_t ref_check = ulTaskNotifyTake(pdTRUE, 5);
		status_led(5, on_led);
		if (queue_get_size(&referee_uart_q) > 7) {
			while (queue_get_size(&referee_uart_q) > 7) {
				proc_status = ref_process_data(&referee_uart_q, &msg_buffer);
				if (proc_status == PROCESS_SUCCESS) {
					switch (msg_buffer.cmd_id) {
					case REF_ROBOT_SHOOT_DATA_CMD_ID:
						memcpy(&ref_shoot_data, &msg_buffer.data,
								sizeof(ref_shoot_data_t));
						ref_shoot_data_txno++;
						break;
					case REF_GAME_STATE_CMD_ID:
							memcpy(&ref_game_state, &msg_buffer.data,
									sizeof(ref_game_state_t));
							ref_game_state_txno++;
							break;
					case REF_ROBOT_DATA_CMD_ID:
						memcpy(&ref_robot_data, &msg_buffer.data,
								sizeof(ref_game_robot_data_t));
						ref_robot_data_txno++;
						break;
					case REF_ROBOT_POS_DATA_CMD_ID:
						memcpy(&ref_robot_pos, &msg_buffer.data,
								sizeof(ref_game_robot_pos_t));
						ref_robot_pos_txno++;
						break;
					case REF_ROBOT_POWER_DATA_CMD_ID:
						memcpy(&ref_power_data, &msg_buffer.data,
								sizeof(ref_robot_power_data_t));
						ref_power_data_txno++;
						break;

					case REF_ROBOT_DMG_DATA_CMD_ID:
						memcpy(&ref_dmg_data, &msg_buffer.data,
								sizeof(ref_robot_dmg_t));
						ref_dmg_data_txno++;
						break;

					case REF_ROBOT_HP_CMD_ID:
						memcpy(&ref_robot_hp, &msg_buffer.data,
								sizeof(ref_game_robot_HP_t));
						ref_robot_hp_txno++;
						break;
					case REF_ROBOT_MAGAZINE_DATA_CMD_ID:
						memcpy(&ref_mag_data, &msg_buffer.data,
								sizeof(ref_magazine_data_t));
						ref_mag_data_txno++;
						//add in the memcpys here
						break;
					default:
						break;
					}
//						if (msg_buffer.cmd_id == REF_ROBOT_SHOOT_DATA_CMD_ID){
//							xQueueSend(uart_data_queue, &msg_buffer, 0);
//						}
				} else if (proc_status == INSUFFICIENT_DATA) {
					break;
				}
			}
		}

		status_led(5, off_led);
		static uint32_t last_sent;

		status_led(5, on_led);
		if (ref_robot_data.robot_level == 1) {
			referee_limiters.feeding_speed = LV1_FEEDER;
			referee_limiters.projectile_speed = LV1_PROJECTILE;
			referee_limiters.robot_level = 1;
			status_led(7, on_led);
			status_led(8, off_led);
		} else if (ref_robot_data.robot_level == 2) {
			referee_limiters.feeding_speed = LV2_FEEDER;
			referee_limiters.projectile_speed = LV2_PROJECTILE;
			referee_limiters.robot_level = 2;
			status_led(7, off_led);
			status_led(8, on_led);
		} else if (ref_robot_data.robot_level == 3) {
			referee_limiters.feeding_speed = LV3_FEEDER;
			referee_limiters.projectile_speed = LV3_PROJECTILE;
			referee_limiters.robot_level = 3;
			status_led(7, on_led);
			status_led(8, on_led);
		} else {
			referee_limiters.feeding_speed = LV1_FEEDER;
			referee_limiters.projectile_speed = LV1_PROJECTILE;
		}
		if (ref_robot_data.robot_level != 0) {
			float temp_buffer = ((((float) ref_power_data.chassis_power_buffer)
					/ 40) + 0.01);
			temp_buffer = (temp_buffer > 1) ? 1 : temp_buffer;
			referee_limiters.wheel_buffer_limit = temp_buffer * temp_buffer;
			float temp_power = ((((float) ref_power_data.chassis_power/(float) ref_robot_data.chassis_power_limit)
					) * 0.5) + 0.5;
			temp_power = (temp_power > 1) ? 1 : temp_power;
			arm_sqrt_f32(temp_power, &referee_limiters.wheel_power_limit);
		} else {
			referee_limiters.wheel_buffer_limit = 1;
			referee_limiters.wheel_power_limit = 1;

		}
		vTaskDelay(10);
	}
}



