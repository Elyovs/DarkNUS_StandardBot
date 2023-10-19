/*
 * bsp_queue.h
 *
 *  Created on: 3 Mar 2022
 *      Author: wx
 */

#ifndef BSP_INC_BSP_QUEUE_H_
#define BSP_INC_BSP_QUEUE_H_
#include "stdint.h"
//library for 8byte queue of size QUEUE_SIZE

#define TQUEUE_SIZE 256

typedef enum {
	Q_OK = 0,
	Q_FULL = 1,
	Q_EMPTY = 2,
	Q_NOT_ENOUGH_BYTES = 3,
	Q_INVALID = 4
}QueueStatus_t;

typedef struct{
	QueueStatus_t op_status;
	uint16_t bytes_appended;
}QueueOpStat_t;

typedef struct {
	uint8_t queue[TQUEUE_SIZE];
	uint16_t last_byte_pos;
	uint16_t curr_byte_pos;
	uint16_t stored_bytes;
	uint32_t last_time;
	uint8_t last_proc;
}queue_t;


QueueOpStat_t queue_init(queue_t* queue);
QueueOpStat_t queue_append_byte(queue_t* queue, uint8_t data);
QueueOpStat_t queue_append_bytes(queue_t* queue, uint8_t *data, uint16_t len);
uint8_t queue_pop_element(queue_t* queue);
QueueOpStat_t queue_pop_elements(queue_t* queue, uint8_t* data, uint16_t len);
uint8_t queue_peek(queue_t* queue);
QueueOpStat_t queue_peek_number(queue_t* queue, uint8_t* buffer,uint16_t size);
QueueOpStat_t queue_remove_number(queue_t* queue, uint16_t size);
uint16_t queue_get_size(queue_t* queue);
uint8_t queue_sanity_check(queue_t* queue);

#endif /* BSP_INC_BSP_QUEUE_H_ */
