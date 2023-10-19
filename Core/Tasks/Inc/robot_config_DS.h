/*
 * robot_config.h
 *
 *  Created on: 19 Jan 2021
 *      Author: Hans Kurnia
 */

#ifndef TASKS_INC_ROBOT_CONFIG_DS_H_
#define TASKS_INC_ROBOT_CONFIG_DS_H_

#define BULLET_17

#define OVERHEAT_PROTECTION
//#define BOARD_DOWN
//0 for SWDIO port to be roll, 1 for SWDIO port to be pitch, 2 for vertical mount SWDIO port to the right
#define IMU_ORIENTATION	0
//doesn't do anything, todo: implement pid for heater
#define IMU_TARGET_TEMP	50

//flip until motor angle and yaw angle matches
#define IMU_YAW_INVERT		-1
#define IMU_PITCH_INVERT	-1
//nothing uses roll.....yet
#define IMU_ROLL_INVERT		1
//#define IST8310
#define CONTROL_DEFAULT			REMOTE_CTRL_MODE

/********************* CONTROL SENSITIVITIES ***********/
#define REMOTE_YAW_SPEED 	 			0.05 			//Speed of gimbal yaw turning
#define REMOTE_PITCH_SPEED 	 		0.05//0.005		//Speed of gimbal pitch turning

#define MOUSE_X_SENSITIVITY		(400 * REMOTE_YAW_SPEED)				//Speed of yaw turning with mouse, dependent on above speed
#define MOUSE_Y_SENSITIVITY 	(200 * REMOTE_PITCH_SPEED)				//Speed of pitch turning with mouse,  dependent on above speed

/*********************** REFEREE SYSTEM CONFIGURATION *******************/
#define FRICTION_SB_SPIN		1000
#define OVERHEAT_TIME			100

#define LV1_FEEDER				480
#define	LV1_PROJECTILE			13.7//`b0//18//20//12
#define LV1_POWER				60
#define LV1_MAX_SPEED			4500
#define LV1_MAX_CURRENT			4000

#define LV2_FEEDER				480
#define	LV2_PROJECTILE			13.7//12
#define LV2_POWER				80
#define LV2_MAX_SPEED			6000
#define LV2_MAX_CURRENT			5000

#define LV3_FEEDER				480
#define	LV3_PROJECTILE			13.7//12
#define LV3_POWER				100
#define LV3_MAX_SPEED			8000
#define LV3_MAX_CURRENT			5500


#define CHASSIS_POWER_MULT		1
//todo: implement power settings lol
//#define MOTOR_CURRENT_RATIO		(CHASSIS_MAX_CURRENT / 10000) 	// 10000 mA at max current
#define PROJECTILE_SPEED_RATIO	360								//rpm per m/s of the friction wheels ish don't think this will work well lmao
#define FEEDER_SPEED_RATIO		8								//projectiles per round of the feeder

/*********************** MANUAL CONTROL CONFIGURATION *******************/
//Inverts for both keyboard and mouse controls
#define YAW_INVERT  			-1				//1 to invert control -1 to disable
#define PITCH_INVERT  			-1				//1 to invert control -1 to disable

#define MOUSE_X_INVERT			1				//Set to -1 if it needs to be inverted
#define	MOUSE_Y_INVERT			-1				//Set to -1 if it needs to be inverted

#define MOUSE_LIMIT 			200

#define KEYBD_MAX_SPD 			1//0.8//0.5				//% of max speed	//% of max speed

#define GIMBAL_MODE 			1				//1 for IMU control, 0 for absolute angle based control

/* PID TUNING GUIDE
 * For all motors, there are 2 different PID values, angle PID and speed PID
 * For motors that require position control, both values have to be set
 *
 * Speed PID is the main control loop which determines how much current
 * to send to the motors. i.e. it sets the target speed for the motors
 * Angle PID calculates the RPM the motor should be running at, then runs the
 * target values through the speed PID loop
 *
 * Generally, the speed control loop should be PID,
 * while the angle control loop can just be a P control
 *
 * TO TUNE PID
 * Tune speed loop FIRST, if not the angle loop might resonate and cause it to oscillate instead
 */

/*********************** LAUNCHER CONFIGURATION ***********************/
#define FEEDER_KP 			7			// |
#define FEEDER_KI  			0.02				// | - FEEDER PID VALUES
#define FEEDER_KD  			3			// |
#define FEEDER_MAX_INT		10000

#define FEEDER_ANGLE_KP 			1			// |
#define FEEDER_ANGLE_KD  			0			// | - FEEDER_ANGLE PID VALUES
#define FEEDER_ANGLE_KI  			0			// |
#define FEEDER_ANGLE_INT_MAX  		0			// |
#define FEEDER_MAX_RPM				100			// |
#define FEEDER_JAM_TORQUE  		9800			//Before feeder deemed to be jammed
#define FEEDER_UNJAM_SPD  		-60				//Reverse unjam
#define FEEDER_UNJAM_TIME		110
#define FEEDER_MAX_CURRENT		10000
#define FEEDER_INVERT			1
#define FEEDER_CUTOFF_TEMP  	60
#define STEPPER_ANGLE			1.8
#define FRICTION_KP  			4//3				// |
#define FRICTION_KI  			0.002//0.02			// | - FRICTION WHEELS PID VALUES
#define FRICTION_KD  			0//10				// |
#define FRICTION_MAX_CURRENT 	16384
#define FRICTION_MAX_INT		10000
#define FRICTION_INVERT			-1
#define LAUNCHER_MARGIN			100
#define LAUNCHER_DIFF_MARGIN	100
#define FRICTION_OFFSET			100

#define CLEAR_DELAY				1000

/*********************** CHASSIS CONFIGURATION ***********************/
#define CHASSIS_KP  		10				// |
#define CHASSIS_KI  		0.1				// | - CHASSIS WHEELS PID VALUES
#define CHASSIS_KD  		5				// |
#define CHASSIS_INT_MAX  	10000				// |
#define CHASSIS_MAX_CURRENT 4000
#define CHASSIS_MIN_CURRENT 0

#define CHASSIS_YAW_MAX_RPM	1.0					//max RPM for chassis centering
#define CHASSIS_YAW_KP 		0.5
#define CHASSIS_YAW_KI		0
#define CHASSIS_YAW_KD 		0//0
#define CHASSIS_TRANS_PRIO		0.5			//% of chassis speed to be prioritised for translation
#define CHASSIS_YAW_PRIO		(1-CHASSIS_TRANS_PRIO)

/* To configure centers, start the boards in debug mode with all motors
 * powered *but in safe mode* (i.e. remotes off)
 * Physically push the motors to the desired centers
 * and put a breakpoint/live expression on their respective real_ang variables
 * from their raw_data structs
 * The centers should be from 0 to 8192, it should be the value directly from
 * the motors
 */
/*********************** GIMBAL CONFIGURATION ***********************/
#define PITCH_ANGLE_KP	  		200
#define PITCH_ANGLE_KD  		00
#define PITCH_ANGLE_KI  		0
#define PITCH_ANGLE_INT_MAX		1000

#define PITCHRPM_KP				200
#define PITCHRPM_KI				0.1
#define PITCHRPM_KD				0
#define PITCHRPM_INT_MAX		10000
#define PITCH_MAX_RPM			200
#define PITCH_MAX_CURRENT		20000

#define PITCH_CENTER			7900
#define PITCH_MAX_ANG			0.6
#define PITCH_MIN_ANG			-0.4

#define YAW_ANGLE_KP			200//200
#define YAW_ANGLE_KI			0
#define YAW_ANGLE_KD			0
#define YAW_ANGLE_INT_MAX		100

#define YAWRPM_KP				100//600//400
#define YAWRPM_KI				0.005
#define YAWRPM_KD				100
#define YAWRPM_INT_MAX			5000
#define YAW_MAX_RPM				400
#define YAW_MAX_CURRENT			20000

#define YAW_CENTER 				4096
#define YAW_MAX_ANG				(PI-0.75)
#define YAW_MIN_ANG				(-PI+0.75)

/*********************** MOTOR CONFIGURATION *******************/
//CAN ids for the motors, for motors on the CAN2 bus, add 12
//ADD 4 TO GM6020 IDS i.e. flashing 5 times = ID 9
//#define CHASSIS_MCU
#ifndef CHASSIS_MCU
#define FL_MOTOR_ID 		13
#define FR_MOTOR_ID 		14
#define BR_MOTOR_ID 		15
#define BL_MOTOR_ID 		16
#endif
#define FEEDER_MOTOR_ID		3
#define LFRICTION_MOTOR_ID	2
#define RFRICTION_MOTOR_ID	1

//NOTE: two motors CANNOT have the same __flashing__ number (i.e. GM6020 id 9 cannot be used
//with any id 6 motors
#define PITCH_MOTOR_ID 		17
#ifndef CHASSIS_MCU
#define YAW_MOTOR_ID 		18
#endif

/* MECANUM WHEEL PROPERTIES */
#define WHEEL_CIRC			7.625	//in CM

#define FR_ANG_X			-PI/4
#define FR_ANG_Y 			-PI/2
#define FR_ANG_PASSIVE		PI/4
#define FR_DIST				312
#define FR_VX_MULT			-1		//-cos(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
#define FR_VY_MULT			-1		//-sin(FR_ANG_Y - FR_ANG_PASSIVE)/sin(FR_ANG_PASSIVE)
#define FR_YAW_MULT			1		//((-FR_DIST * sin(FR_ANG_Y - FR_ANG_PASSIVE - FR_ANG_X)) / (sin(FR_ANG_PASSIVE) * WHEEL_CIRC))

#define FL_ANG_X			PI/4
#define FL_ANG_Y 			PI/2
#define FL_ANG_PASSIVE		-PI/4
#define FL_DIST				312
#define FL_VX_MULT			-1 		//-cos(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
#define FL_VY_MULT			1		//-sin(FL_ANG_Y - FL_ANG_PASSIVE)/sin(FL_ANG_PASSIVE)
#define FL_YAW_MULT			1	//((-FL_DIST * sin(FL_ANG_Y - FL_ANG_PASSIVE - FL_ANG_X)) / (sin(FL_ANG_PASSIVE) * WHEEL_CIRC))

#define BL_ANG_X			(3*PI/4)
#define BL_ANG_Y 			PI/2
#define BL_ANG_PASSIVE		PI/4
#define BL_DIST				312
#define BL_VX_MULT			1		//-cos(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
#define BL_VY_MULT			1		//-sin(BL_ANG_Y - BL_ANG_PASSIVE)/sin(BL_ANG_PASSIVE)
#define BL_YAW_MULT			1	//((-BL_DIST * sin(BL_ANG_Y - BL_ANG_PASSIVE - BL_ANG_X)) / (sin(BL_ANG_PASSIVE) * WHEEL_CIRC))

#define BR_ANG_X			-(3*PI/4)
#define BR_ANG_Y 			-PI/2
#define BR_ANG_PASSIVE		-PI/4
#define	BR_DIST				312
#define BR_VX_MULT			1		//-cos(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
#define BR_VY_MULT			-1		//-sin(BR_ANG_Y - BR_ANG_PASSIVE)/sin(BR_ANG_PASSIVE)
#define BR_YAW_MULT			1	//((-BR_DIST * sin(BR_ANG_Y - BR_ANG_PASSIVE - BR_ANG_X)) / (sin(BR_ANG_PASSIVE) * WHEEL_CIRC))

/*********************** OTHERS ***********************/

#define CONTROL_DELAY 			2
#define GIMBAL_DELAY			5
#define CHASSIS_DELAY 			10

//microsecond timer used for PIDs
#define TIMER_FREQ			10000 //Cannot be too high if not the ISRs overload the CPU
#endif /* TASKS_INC_ROBOT_CONFIG_DS_H_ */
