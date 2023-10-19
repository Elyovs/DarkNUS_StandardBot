/*
 * rtos_g_vars.h
 *
 *  Created on: 23 Feb 2023
 *      Author: wx
 */

#ifndef TASKS_INC_RTOS_G_VARS_H_
#define TASKS_INC_RTOS_G_VARS_H_


extern TaskHandle_t gimbal_control_task_handle;
extern TaskHandle_t movement_control_task_handle;
extern TaskHandle_t referee_processing_task_handle;
extern TaskHandle_t control_input_task_handle;
extern TaskHandle_t launcher_control_task_handle;
extern TaskHandle_t buzzing_task_handle;
extern TaskHandle_t motor_calib_task_handle;
extern TaskHandle_t usb_task_handle;
extern TaskHandle_t imu_processing_task_handle;

extern EventGroupHandle_t gimbal_event_group;
extern EventGroupHandle_t chassis_event_group;
extern EventGroupHandle_t launcher_event_group;

extern SemaphoreHandle_t usb_continue_semaphore;

extern QueueHandle_t gyro_data_queue;
extern QueueHandle_t accel_data_queue;
extern QueueHandle_t mag_data_queue;

extern QueueHandle_t telem_data_queue;
extern QueueHandle_t buzzing_task_msg;
extern QueueHandle_t xvr_data_queue;
extern QueueHandle_t uart_data_queue;
#endif /* TASKS_INC_RTOS_G_VARS_H_ */
