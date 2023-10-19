/*
 * bsp_queue.c
 *
 *  Created on: 3 Mar 2022
 *      Author: wx
 */
#include "bsp_queue.h"
#include <string.h>



QueueOpStat_t queue_init(queue_t* queue){
	QueueOpStat_t op_stat;
	op_stat.op_status = Q_OK;
	if (queue == NULL){
		op_stat.op_status = Q_INVALID;
		return op_stat;
	}
	queue->curr_byte_pos = 0;
	queue->last_byte_pos=0;
	queue->last_proc = 0;
	queue->last_time=0;
	queue->stored_bytes=0;
	return op_stat;
}

/*
 * Adds a byte to the end of the byte_queue
 * Add one byte at a time!
 */
QueueOpStat_t queue_append_byte(queue_t* queue, uint8_t data){
	QueueOpStat_t op_stat;
	op_stat.op_status = Q_OK;
	queue->last_time = HAL_GetTick();
	queue->queue[queue->last_byte_pos] = data;
	op_stat.bytes_appended = 1;
	queue->last_byte_pos = (queue->last_byte_pos >= TQUEUE_SIZE-1) ? 0 : queue->last_byte_pos + 1;
	if (queue->stored_bytes > TQUEUE_SIZE) {
		queue->stored_bytes = TQUEUE_SIZE;
		queue->curr_byte_pos = (queue->curr_byte_pos == TQUEUE_SIZE-1) ? 0 : queue->curr_byte_pos+1;
		op_stat.op_status = Q_FULL;
		return op_stat;
	}
	queue->stored_bytes+=1;
	return op_stat;
}

QueueOpStat_t queue_append_bytes(queue_t* queue, uint8_t *data, uint16_t len){

	QueueOpStat_t op_stat;
	op_stat.op_status = Q_OK;
	//check distance until end, copy all if have space
	//else copy until last byte in queue, loop back to front
	//change starting byte if queue was full
	//check whether the user is trying to put in more bytes
	//than the queue can hold
	if (len / TQUEUE_SIZE > 0){
		//WHY are you putting in more data than the queue buffer
		data += (len-TQUEUE_SIZE);
		len = TQUEUE_SIZE;
		op_stat.op_status = Q_FULL;
	}
	queue->stored_bytes+= len;

	//check if it has to loop around the array
	if ((TQUEUE_SIZE-queue->last_byte_pos)-len < 0){
		uint16_t len_til_end = TQUEUE_SIZE-queue->last_byte_pos;
		if (len_til_end > 0){
			memcpy(queue->queue+queue->last_byte_pos,data,len_til_end);
		}
		uint16_t remaining_len = len-len_til_end;
		memcpy(queue->queue,data+len_til_end,remaining_len);
		queue->last_byte_pos = remaining_len-1;
	} else {
		memcpy(queue->queue+queue->last_byte_pos, data, len);
		queue->last_byte_pos += len;
	}
	//loop back if it's at the last position
	if (queue->last_byte_pos >= TQUEUE_SIZE){
		queue->last_byte_pos = queue->last_byte_pos-TQUEUE_SIZE;
	}
	//check if queue has overflowed
	if (queue->stored_bytes >= TQUEUE_SIZE){
		uint16_t new_offset = queue->stored_bytes-TQUEUE_SIZE;
		queue->curr_byte_pos += new_offset;
		op_stat.op_status = Q_FULL;
		//check if queue has looped back
		if (queue->curr_byte_pos > TQUEUE_SIZE-1){
			queue->curr_byte_pos = queue->curr_byte_pos- TQUEUE_SIZE;
		}
		queue->stored_bytes = TQUEUE_SIZE;
	}
	op_stat.bytes_appended = len;
	return op_stat;
}

uint8_t queue_pop_element(queue_t* queue){

	if (queue->stored_bytes == 0){
		return 0;
	} else {
		uint8_t temp =queue->queue[queue->curr_byte_pos];
		queue->curr_byte_pos +=1;
		queue->curr_byte_pos = (queue->curr_byte_pos >= TQUEUE_SIZE) ?
				0:queue->curr_byte_pos;
		queue->stored_bytes -= 1;
		return temp;
	}
}


QueueOpStat_t queue_pop_elements(queue_t* queue, uint8_t* data, uint16_t len){
		QueueOpStat_t op_stat;

		op_stat = queue_peek_number(queue, data, len);
		QueueOpStat_t remove_stat;
		remove_stat = queue_remove_number(queue, op_stat.bytes_appended);
	return op_stat;
}

uint8_t queue_peek(queue_t* queue){
	if (queue->stored_bytes>0){
		return queue->queue[queue->curr_byte_pos];
	}
	else{
		return 0;
	}
}

QueueOpStat_t queue_peek_number(queue_t* queue, uint8_t* buffer,uint16_t size){
	QueueOpStat_t op_stat;
	queue->last_time = HAL_GetTick();
	op_stat.op_status = Q_OK;
	uint16_t bytes_to_end = TQUEUE_SIZE-queue->curr_byte_pos;
	if (queue->stored_bytes < size){
		op_stat.op_status = Q_NOT_ENOUGH_BYTES;
		size = queue->stored_bytes;
	}

	if (bytes_to_end >= size){
		memcpy(buffer, &queue->queue[queue->curr_byte_pos], size);
	} else {
		memcpy(buffer, &queue->queue[queue->curr_byte_pos], bytes_to_end);
		memcpy(buffer+bytes_to_end, queue->queue,size-bytes_to_end);
	}
	op_stat.bytes_appended = size;
	return op_stat;
}

QueueOpStat_t queue_remove_number(queue_t* queue, uint16_t size){
//	queue_sanity_check(queue);
	QueueOpStat_t op_stat;
	op_stat.op_status = Q_OK;
	if (queue->stored_bytes < size){
		op_stat.op_status = Q_NOT_ENOUGH_BYTES;
		size = queue->stored_bytes;
	}

	queue->curr_byte_pos += size;
	queue->stored_bytes -= size;
	queue->curr_byte_pos = (queue->curr_byte_pos >= TQUEUE_SIZE) ?
			queue->curr_byte_pos-TQUEUE_SIZE : queue->curr_byte_pos;
	op_stat.bytes_appended = size;
	return op_stat;
}

uint16_t queue_get_size(queue_t* queue){
	return queue->stored_bytes;
}

uint8_t queue_sanity_check(queue_t* queue){
	//check if the queue values are still ok
	if (queue->stored_bytes > TQUEUE_SIZE){
		return 1;
	}
	if (queue->last_byte_pos >= TQUEUE_SIZE || queue->last_byte_pos < 0){
		return 2;
	}
	if (queue->curr_byte_pos >= TQUEUE_SIZE || queue->curr_byte_pos < 0){
		return 3;
	}

	int16_t queue_real_size = queue->last_byte_pos - queue->curr_byte_pos;
	queue_real_size = (queue_real_size < 0) ? TQUEUE_SIZE + queue_real_size : queue_real_size;
	if (queue->stored_bytes == TQUEUE_SIZE){
				return 0;
	}
	if (queue_real_size != queue->stored_bytes){
		return 4;
	}
	return 0;
}



