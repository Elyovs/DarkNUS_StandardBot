/*
 * imu_processing_task.h
 *
 *  Created on: 24 Jan 2022
 *      Author: wx
 */

#ifndef TASKS_INC_IMU_PROCESSING_TASK_H_
#define TASKS_INC_IMU_PROCESSING_TASK_H_

void imu_proc_task_notif();
void gyro_data_ready(gyro_data_t gyro_data);
void accel_data_ready(accel_data_t accel_data);
void mag_data_ready(mag_data_t mag_data);
void imu_processing_task(void *argument);
void init_quaternion(void);
void imu_ahrs_update(void);
void imu_attitude_update(void);
#endif /* TASKS_INC_IMU_PROCESSING_TASK_H_ */
