/*
 * launcher_control_task.c
 *
 *  Created on: Jul 26, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "launcher_control_task.h"

extern EventGroupHandle_t launcher_event_group;

extern ref_game_state_t ref_game_state;
extern motor_data_t can_motors[24];
extern gun_control_t launcher_ctrl_data;

extern remote_cmd_t remote_cmd;
extern referee_limit_t referee_limiters;

static uint32_t start_time = 0;
static uint32_t clear_time = 0;
static uint8_t unjamming = 0;

static uint8_t overheat = 0;
//static float friction_offset = FRICTION_OFFSET;

extern QueueHandle_t telem_motor_queue;

#define BULLET_17_HEAT 10
#define BULLET_42_HEAT 100

#ifdef BULLET_17
#define BULLET_ACTUAL_HEAT BULLET_17_HEAT
#endif

#ifdef BULLET_42
#define BULLET_ACTUAL_HEAT BULLET_42_HEAT
#endif

extern ref_game_robot_data_t ref_robot_data;
extern ref_robot_power_data_t ref_power_data;

extern uint32_t ref_power_data_txno;
extern ref_magazine_data_t ref_mag_data;
extern uint32_t ref_mag_data_txno;
static uint32_t prev_power_data_no = 0;
static uint32_t prev_mag_data_no = 0;

void launcher_control_task(void *argument) {
	TickType_t start_time;
	while (1) {
		//event flags!
		xEventGroupWaitBits(launcher_event_group, 0b111, pdTRUE, pdTRUE,
		portMAX_DELAY);
		status_led(4, on_led);
		start_time = xTaskGetTickCount();

		if (launcher_ctrl_data.enabled) {
			launcher_control(can_motors + LFRICTION_MOTOR_ID - 1,
					can_motors + RFRICTION_MOTOR_ID - 1,
					can_motors + FEEDER_MOTOR_ID - 1);

		} else {
			can_motors[LFRICTION_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[RFRICTION_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[FEEDER_MOTOR_ID - 1].rpm_pid.output = 0;
			motor_send_can(can_motors, FEEDER_MOTOR_ID, LFRICTION_MOTOR_ID,
			RFRICTION_MOTOR_ID, 0);
		}
		status_led(4, off_led);
		//vTaskDelay(CHASSIS_DELAY);
		xEventGroupClearBits(launcher_event_group, 0b111);
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}

}

void launcher_control(motor_data_t *left_friction_motor,
		motor_data_t *right_friction_motor, motor_data_t *feeder) {

	int16_t feeder_output = 0;
	static uint32_t overheat_time;
	static float target_ang;
	uint32_t curr_time = HAL_GetTick();
	static uint32_t overheat_start;
	static uint32_t last_fire;
	static uint8_t fired;
	int16_t firing_speed = launcher_ctrl_data.gun_feeding_speed
			* referee_limiters.feeding_speed / FEEDER_SPEED_RATIO;

	if (launcher_ctrl_data.gun_feeding_speed == 0) {
		feeder->rpm_pid.output = 0;
		speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid);
		left_friction_motor->rpm_pid.output = 0;
		right_friction_motor->rpm_pid.output = 0;
	} else {
		int16_t launcher_rpm = -referee_limiters.projectile_speed
				* FRICTION_INVERT * PROJECTILE_SPEED_RATIO;

		speed_pid(launcher_rpm, left_friction_motor->raw_data.rpm,
				&left_friction_motor->rpm_pid);
		speed_pid(launcher_rpm, right_friction_motor->raw_data.rpm,
				&right_friction_motor->rpm_pid);
		speed_pid(firing_speed * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
	}

	motor_send_can(can_motors, FEEDER_MOTOR_ID, LFRICTION_MOTOR_ID,
	RFRICTION_MOTOR_ID, 0);
}

