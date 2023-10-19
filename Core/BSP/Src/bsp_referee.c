/*
 * bsp_referee.c
 *
 *  Created on: Jun 28, 2021
 *      Author: wx
 */

#include "bsp_queue.h"
#include "board_lib.h"
#include "bsp_referee.h"



enum ref_proc_state{
	HEADER,
	DATA
};


ref_processing_status_t ref_process_data(queue_t *uart_queue, ref_msg_t *proc_msg){
	static uint8_t state = 0;
	static ref_frame_header_t header;
	while(queue_get_size(uart_queue) > REF_HEADER_SIZE){
		switch(state){
			case HEADER:
				if (queue_get_size(uart_queue)< REF_HEADER_SIZE+2 ){
					return INSUFFICIENT_DATA;
				}
				if (queue_peek(uart_queue) != 0xA5){
					queue_pop_element(uart_queue);
				}else {
					if (queue_get_size(uart_queue) > REF_HEADER_SIZE+2){
						header = ref_get_header(uart_queue);
						if (header.cmd_id == 0xFFFF){
							//invalid data, continue checking
							break;
						}
						//header is valid, remove header bytes and start finding data
						state = DATA;
					} else {
						return INSUFFICIENT_DATA;
					}
				}
				break;
			case DATA:
				//wait until enough data to fill data
				if (queue_get_size(uart_queue) > header.data_length+4+REF_HEADER_SIZE){
					state = HEADER;
					*proc_msg = ref_get_msg(header,uart_queue);
					if (proc_msg->cmd_id!= 0xFFFF){
						return PROCESS_SUCCESS;
					} else {
						break;
					}
				} else{
					return INSUFFICIENT_DATA;
				}
				break;
			}
	}
	return INSUFFICIENT_DATA;
}

ref_frame_header_t ref_get_header(queue_t *data_buffer){
	ref_frame_header_t ret_header;
	//+2 for cmd ID
	uint8_t temp_data[REF_HEADER_SIZE+2];
	queue_peek_number(data_buffer, temp_data, 7);
	ret_header.start_frame = temp_data[0];
	if (ret_header.start_frame!= 0xA5){
		queue_pop_element(data_buffer);
		ret_header.cmd_id=0xFFFF;
		return ret_header;
	}

	ret_header.data_length = ((uint16_t) temp_data[2] << 8 | temp_data[1]);
	ret_header.seq = temp_data[3];
	ret_header.crc = temp_data[4];
	int crc_check = verify_CRC8_check_sum(temp_data, 5);
	ret_header.cmd_id = (temp_data[6]) << 8 | temp_data[5];
	if (!crc_check || ret_header.data_length > TQUEUE_SIZE || ret_header.cmd_id > 0x305) {
		ret_header.cmd_id=0xFFFF;
		queue_pop_element(data_buffer);
		return ret_header;
	}
//	queue_remove_number(data_buffer, REF_HEADER_SIZE);
	return ret_header;
}



ref_msg_t ref_get_msg(ref_frame_header_t header,queue_t *uart_queue) {
	uint8_t temp_buffer[TQUEUE_SIZE];
	ref_msg_t buffer_msg;
	//+ 2 for CRC16
	queue_pop_elements(uart_queue, temp_buffer, header.data_length+REF_HEADER_SIZE+4);

	uint32_t crc_test = verify_CRC16_check_sum(temp_buffer, header.data_length+REF_HEADER_SIZE+4);
	if (!crc_test){
		buffer_msg.cmd_id = 0xFFFF;
		return buffer_msg;
	}
	//shift 2 bytes to the right to get pointer to the actual start of data
	uint8_t* data_buffer = temp_buffer+2+REF_HEADER_SIZE;
	buffer_msg.cmd_id = header.cmd_id;
	switch (header.cmd_id) {
	case REF_GAME_STATE_CMD_ID:
		memcpy(&buffer_msg.data.game_state, 	(data_buffer), sizeof(ref_game_state_t));
		break;
	case REF_GAME_RESULT_ID:
		memcpy(&buffer_msg.data.game_result, 	(data_buffer), sizeof(ref_game_result_t));
		break;
	case REF_ROBOT_HP_CMD_ID:
		memcpy(&buffer_msg.data.robot_hp, 		(data_buffer), sizeof(ref_game_robot_HP_t));
		break;
	case REF_DART_LAUNCH_STATUS_CMD_ID:
		memcpy(&buffer_msg.data.dart_status, 	(data_buffer), sizeof(ref_dart_status_t));
		break;
	case REF_GAME_EVENT_CMD_ID:
		memcpy(&buffer_msg.data.game_event, 	(data_buffer), sizeof(ref_game_event_data_t));
		break;
	case REF_SUPPLIER_STATUS_CMD_ID:
		memcpy(&buffer_msg.data.projectile_supply_state, (data_buffer),sizeof(ref_supply_projectile_data_t));
		break;
	case REF_SUPPLIER_BOOKING_CMD_ID:
		memcpy(&buffer_msg.data.projectile_supply_queue, (data_buffer),sizeof(ref_supply_projectile_booking_t));
		break;
	case REF_FOUL_CMD_ID:
		memcpy(&buffer_msg.data.referee_warning,(data_buffer), sizeof(ref_referee_warning_t));
		break;
	case REF_DART_COOLDOWN_CMD_ID:
		memcpy(&buffer_msg.data.dart_cooldown, 	(data_buffer), sizeof(ref_dart_cooldown_t));
		break;
	case REF_ROBOT_DATA_CMD_ID:
		memcpy(&buffer_msg.data.robot_state, 	(data_buffer), sizeof(ref_game_robot_data_t));
		break;
	case REF_ROBOT_POWER_DATA_CMD_ID:
		memcpy(&buffer_msg.data.power_data, 	(data_buffer), sizeof(ref_robot_power_data_t));
		break;
	case REF_ROBOT_POS_DATA_CMD_ID:
		memcpy(&buffer_msg.data.robot_pos, 		(data_buffer), sizeof(ref_game_robot_pos_t));
		break;
	case REF_ROBOT_BUFF_DATA_CMD_ID:
		memcpy(&buffer_msg.data.robot_buff, 	(data_buffer), sizeof(ref_buff_data_t));
		break;
	case REF_AERIAL_ENERGY_DATA_CMD_ID:
		memcpy(&buffer_msg.data.aerial_energy, 	(data_buffer), sizeof(ref_aerial_robot_energy_t));
		break;
	case REF_ROBOT_DMG_DATA_CMD_ID:
		memcpy(&buffer_msg.data.damage_data, 	(data_buffer), sizeof(ref_robot_dmg_t));
		break;
	case REF_ROBOT_SHOOT_DATA_CMD_ID:
		memcpy(&buffer_msg.data.shooting_data, 	(data_buffer), sizeof(ref_shoot_data_t));
		break;
	case REF_ROBOT_MAGAZINE_DATA_CMD_ID:
		memcpy(&buffer_msg.data.magazine_data, 	(data_buffer), sizeof(ref_magazine_data_t));
		break;
	case REF_ROBOT_RFID_BUFF_DATA_CMD_ID:
		memcpy(&buffer_msg.data.rfid_buff, 		(data_buffer), sizeof(ref_rfid_status_t));
		break;
	case REF_DART_STATUS_CMD_ID:
		memcpy(&buffer_msg.data.dart_cmd, 		(data_buffer), sizeof(ref_dart_status_t));
		break;
	case REF_ROBOT_COMMS_CMD_ID:
		memcpy(&buffer_msg.data.robot_comms, 	(data_buffer), sizeof(ref_inter_robot_data_t));
		break;
	case REF_CUSTOM_DATA_CMD_ID:
		memcpy(&buffer_msg.data.custom_data, 	(data_buffer), sizeof(ref_custom_data_t));
		break;
	default:
		buffer_msg.cmd_id = 0xFFFF;
		break;
	}
	//remove cmd id + data
	return buffer_msg;
}


