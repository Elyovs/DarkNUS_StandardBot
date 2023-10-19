/*
 * bsp_referee.h
 *
 *  Created on: Jun 28, 2021
 *      Author: wx
 */

#ifndef BSP_INC_BSP_REFEREE_H_
#define BSP_INC_BSP_REFEREE_H_
#include "referee_msgs.h"

/*frame header 	5 bytes
 * cmd_id		2 bytes
 * data			28 bytes		| assuming we're not transmitting data from robot to robot, maximum is 28 bytes for robot hp data
 * frame tail	2 bytes
 * total: 37 bytes
 */
typedef enum {
	PROCESS_SUCCESS=0,
	INSUFFICIENT_HEADER=1,
	INSUFFICIENT_DATA=2,
	WRONG_HEADER_CRC=3,
	WRONG_DATA_CRC=4

}ref_processing_status_t;

ref_processing_status_t ref_process_data(queue_t *uart_queue, ref_msg_t *proc_msg);
ref_frame_header_t ref_get_header(queue_t *data_buffer);
ref_msg_t ref_get_msg(ref_frame_header_t header,queue_t *uart_queue);



#endif /* BSP_INC_BSP_REFEREE_H_ */
