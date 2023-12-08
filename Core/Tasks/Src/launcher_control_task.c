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
		feeder_output = 0;

		if (launcher_ctrl_data.override == 1) {
			speed_pid(
					referee_limiters.projectile_speed * FRICTION_INVERT
							* PROJECTILE_SPEED_RATIO,
					left_friction_motor->raw_data.rpm,
					&left_friction_motor->rpm_pid);
			speed_pid(
					-referee_limiters.projectile_speed * FRICTION_INVERT
							* PROJECTILE_SPEED_RATIO,
					right_friction_motor->raw_data.rpm,
					&right_friction_motor->rpm_pid);

		} else if (clear_time + CLEAR_DELAY > HAL_GetTick()) {
			speed_pid(
					referee_limiters.projectile_speed * FRICTION_INVERT
							* PROJECTILE_SPEED_RATIO,
					left_friction_motor->raw_data.rpm,
					&left_friction_motor->rpm_pid);
			speed_pid(
					-referee_limiters.projectile_speed * FRICTION_INVERT
							* PROJECTILE_SPEED_RATIO,
					right_friction_motor->raw_data.rpm,
					&right_friction_motor->rpm_pid);
			if (unjamming == 1) { // if unjam time reached, or jam on reverse torque
				if ((start_time + FEEDER_UNJAM_TIME < HAL_GetTick())
						|| (feeder->raw_data.torque < -FEEDER_JAM_TORQUE)) {
					unjamming = 0;
#ifndef ANGLE_FEEDER
					feeder_output = firing_speed;
#endif
				} else {
					feeder_output = FEEDER_UNJAM_SPD;
				}
			} else {
#ifndef ANGLE_FEEDER
				feeder_output = firing_speed;
;
#endif
			}

		} else if (ref_game_state.game_progress == 4){

			speed_pid(FRICTION_SB_SPIN * FRICTION_INVERT,
					left_friction_motor->raw_data.rpm,
					&left_friction_motor->rpm_pid);
			speed_pid(
					-FRICTION_SB_SPIN * FRICTION_INVERT,
					right_friction_motor->raw_data.rpm,
					&right_friction_motor->rpm_pid);

		}else {
			left_friction_motor->rpm_pid.output = 0;
			right_friction_motor->rpm_pid.output = 0;
		}
		overheat_time = curr_time + 1000;
		overheat = 0;
	} else {

#ifdef OVERHEAT_PROTECTION
		if (overheat == 1) {
			if (curr_time - overheat_start >= OVERHEAT_TIME) {
				overheat = 0;
			}
		}
		if (prev_power_data_no != ref_power_data_txno) {
			prev_power_data_no = ref_power_data_txno;
//			int16_t cur_fire_rate = fabs(launcher_ctrl_data.gun_feeding_speed* referee_limiters.feeding_speed);
			int16_t ammo_remaining;

#ifdef BULLET_17
			uint8_t active_feeder = 2;
			if (ref_robot_data.shooter17_heat1_cooling_limit == 0
					&& ref_robot_data.shooter17_heat0_cooling_limit == 0) {
				active_feeder = 3;
			} else if (ref_robot_data.shooter17_heat0_cooling_limit == 0) {
				active_feeder = 1;
			} else if (ref_robot_data.shooter17_heat1_cooling_limit == 0) {
				active_feeder = 0;
			}

			if (active_feeder == 2) {
				if (ref_power_data.shooter_heat0
						>= ref_power_data.shooter_heat1) {
					active_feeder = 0;
				} else {
					active_feeder = 1;
				}

			}

			if (active_feeder == 0) {
				ammo_remaining =
						((((int16_t) ref_robot_data.shooter17_heat0_cooling_limit
								- (int16_t) ref_power_data.shooter_heat0))
								/ BULLET_17_HEAT) - 2;
			} else if (active_feeder == 1) {
				ammo_remaining =
						((((int16_t) ref_robot_data.shooter17_heat1_cooling_limit
								- (int16_t) ref_power_data.shooter_heat1))
								/ BULLET_17_HEAT) - 2;
			}
			if (active_feeder == 3) {
				ammo_remaining = 1000;
			}
#ifdef CHECK_AMMO
			if (ammo_remaining > ref_mag_data.magazine_17mm) {
				ammo_remaining = ref_mag_data.magazine_17mm;
			}

#endif
#endif

#ifdef BULLET_42
//			ammo_remaining = (ref_robot_data.shooter42_heat1_cooling_limit
//			- ref_power_data.shooter_heat0)/ BULLET_42_HEAT;
//			ammo_remaining -=1;
			ammo_remaining = 100;

#ifdef CHECK_AMMO
			if (ammo_remaining > ref_mag_data.magazine_42mm){
				ammo_remaining = ref_mag_data.magazine_42mm;
			}
#endif

#endif

			if (ammo_remaining < 3) {
				overheat = 1;
				overheat_start = curr_time;
			} else {
				if (ammo_remaining < 10){
					firing_speed = firing_speed * ammo_remaining/10;
				}
			}
			overheat_time = curr_time
					+ (ammo_remaining * 60 * 1000) / (firing_speed) - 50;
		}
#endif

		//TODO: add in speed checks and shtuffasdwqwe
		clear_time = HAL_GetTick();
		speed_pid(
				referee_limiters.projectile_speed * FRICTION_INVERT
						* PROJECTILE_SPEED_RATIO,
				left_friction_motor->raw_data.rpm,
				&left_friction_motor->rpm_pid);
		speed_pid(
				-referee_limiters.projectile_speed * FRICTION_INVERT
						* PROJECTILE_SPEED_RATIO,
				right_friction_motor->raw_data.rpm,
				&right_friction_motor->rpm_pid);
//		left_friction_motor->rpm_pid.output+=friction_offset;
//		right_friction_motor->rpm_pid.output-=friction_offset;

#ifdef ANGLE_FEEDER
		if (curr_time - last_fire > 100) {
			target_ang = feeder->angle_data.adj_ang
					+ (2 * PI / FEEDER_SPEED_RATIO);
			if (target_ang > PI) {
				target_ang -= 2 * PI;
			} else if (target_ang < -PI) {
				target_ang += 2 * PI;
			}

		}
		last_fire = curr_time;
#endif

		if (((((fabs(left_friction_motor->raw_data.rpm)
				- fabs(
						(launcher_ctrl_data.projectile_speed
								* PROJECTILE_SPEED_RATIO
								* referee_limiters.projectile_speed))
				< LAUNCHER_MARGIN)))
				&& ((fabs(right_friction_motor->raw_data.rpm)
						- fabs(
								(launcher_ctrl_data.projectile_speed
										* PROJECTILE_SPEED_RATIO
										* referee_limiters.projectile_speed))
						< LAUNCHER_MARGIN)))
				&& ((fabs(right_friction_motor->raw_data.rpm)
						- fabs(left_friction_motor->raw_data.rpm))
						< LAUNCHER_DIFF_MARGIN)) {

			if ((feeder->raw_data.torque > FEEDER_JAM_TORQUE)) {
				unjamming = 1;
				start_time = HAL_GetTick();
			}

			if (unjamming == 1) { // if unjam time reached, or jam on reverse torque
				if ((start_time + FEEDER_UNJAM_TIME < HAL_GetTick())
						|| (feeder->raw_data.torque < -FEEDER_JAM_TORQUE)) {
					unjamming = 0;
#ifndef ANGLE_FEEDER
					feeder_output = firing_speed;
#endif
				} else {
					feeder_output = FEEDER_UNJAM_SPD;
				}
			} else {
#ifndef ANGLE_FEEDER
				feeder_output = firing_speed;
;
#endif
			}

		}
	}

	if ((overheat == 1) && launcher_ctrl_data.override == 0) {
		feeder_output = 0;
	}

	if (unjamming == 1) { // if unjam time reached, or jam on reverse torque
		if ((start_time + FEEDER_UNJAM_TIME < HAL_GetTick())
				|| (feeder->raw_data.torque < -FEEDER_JAM_TORQUE)) {
			unjamming = 0;
		}
	}

#ifdef ANGLE_FEEDER
	if (unjamming == 0) {
		float ang_diff = target_ang - feeder->angle_data.adj_ang;
		ang_diff = (ang_diff > PI) ? ang_diff - 2 * PI :
					(ang_diff < -PI) ? ang_diff + 2 * PI : ang_diff;
		feeder_output = ang_diff * FEEDER_ANGLE_KP;
		feeder_output = (feeder_output > FEEDER_MAX_RPM) ?
		FEEDER_MAX_RPM :
															feeder_output;
		feeder_output =
				(feeder_output < -FEEDER_MAX_RPM) ?
						-FEEDER_MAX_RPM : feeder_output;
	}
#endif

	if (feeder_output == 0) {
//		feeder->rpm_pid.output = 0;
		speed_pid(0, feeder->raw_data.rpm, &feeder->rpm_pid);
	} else {
		speed_pid(feeder_output * feeder->angle_data.gearbox_ratio,
				feeder->raw_data.rpm, &feeder->rpm_pid);
		//speed_pid(feeder_output * 36,feeder->raw_data.rpm, &feeder->rpm_pid);
	}

	motor_send_can(can_motors, FEEDER_MOTOR_ID, LFRICTION_MOTOR_ID,
	RFRICTION_MOTOR_ID, 0);
}

