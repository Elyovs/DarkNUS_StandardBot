/*
 * typedefs.h
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#ifndef TASKS_INC_TYPEDEFS_H_
#define TASKS_INC_TYPEDEFS_H_
//switch goes from 1-3-2 from top to down
enum left_switch
{
	ge_LSW_UNSAFE = 1,
	ge_LSW_CONFIG = 3,
	ge_LSW_STANDBY = 2
};

enum right_switch
{
	ge_RSW_SHUTDOWN = 1,
	ge_RSW_GIMBAL = 3,
	ge_RSW_ALL_ON = 2,
};

#define KEYBOARD_CTRL_MODE	1
#define REMOTE_CTRL_MODE	2
#define SBC_CTRL_MODE 		3

enum rb_mode_t{
	ge_RB_KEYBOARD,
	ge_RB_REMOTE,
	ge_RB_SBC_CTRL
};

typedef struct
{
	float kp;
	float ki;
	float kd;
	float error[2];
	float integral;
	float int_max;
	float max_out;
	float output;
	float physical_max;
	uint32_t last_time[2];
}pid_data_t;

typedef struct	{
	int32_t ticks;
	int32_t center_ang;
	float min_ang;
	float max_ang;
	float phy_min_ang;
	float phy_max_ang;
	float gearbox_ratio;
	float adj_ang;
	float dist;
	float wheel_circ; //in cm
	float hires_rpm;
} angle_data_t;

typedef struct {
	int16_t angle[2];
	int16_t rpm;
	int16_t torque;
	uint8_t temp;
}raw_data_t;

typedef struct {
	uint16_t id;
	uint8_t motor_type;
	raw_data_t raw_data;
	pid_data_t rpm_pid;
	pid_data_t angle_pid;
	angle_data_t angle_data;
	uint32_t last_time[2];
} motor_data_t;


/* Struct containing cleaned data from remote */
typedef struct {
	/* Joysticks - Values range from -660 to 660 */
	int16_t right_x;
	int16_t right_y;
	int16_t left_x;
	int16_t left_y;
	/* Switches - Values range from 1 - 3 */
	int8_t left_switch;
	int8_t right_switch;
	/* Mouse movement - Values range from -32768 to 32767 */
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int32_t mouse_hori;
	int32_t mouse_vert;
	/* Mouse clicks - Values range from 0 to 1 */
	int8_t mouse_left;
	int8_t mouse_right;

	/* Keyboard keys mapping
	 * Bit0 -- W 键
	 * Bit1 -- S 键
	 *	Bit2 -- A 键
	 *	Bit3 -- D 键
	 *	Bit4 -- Q 键
	 *	Bit5 -- E 键
	 *	Bit6 -- Shift 键
	 *	Bit7 -- Ctrl 键
	 *
	 */
	uint16_t keyboard_keys;
	int16_t side_dial;
	uint32_t last_time;
} remote_cmd_t;



typedef struct
{
	float gx;
	float gy;
	float gz;
	uint32_t last_gyro_update;
}gyro_data_t;

typedef struct
{
	float ax;
	float ay;
	float az;
	uint32_t last_accel_update;
}accel_data_t;

typedef struct
{
	int16_t mx;
	int16_t my;
	int16_t mz;
	uint32_t last_mag_update;
}mag_data_t;


typedef struct
{
	gyro_data_t gyro_data;
	accel_data_t accel_data;
	mag_data_t mag_data;

	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} imu_raw_t;


typedef struct{
	float ax;
	float ay;
	float az;
}linear_accel_t;

typedef struct
{
	float pit;
	float rol;
	float yaw;
} orientation_data_t;

typedef struct
{
	float temp;

	float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float wy;
	float wz;

	float vx;
	float vy;
	float vz;

	float gx;
	float gy;
	float gz;

} imu_processor_t;

typedef struct
{
	int16_t feeding_speed;
	int16_t projectile_speed;
	float wheel_power_limit;
	float wheel_buffer_limit;
	uint8_t robot_level;
	float chassis_power;
	uint32_t last_update_time;

}referee_limit_t;


typedef struct
{
	float pitch;
	float yaw;
	uint8_t imu_mode;
	uint8_t enabled;
}gimbal_control_t;


typedef struct
{
	float forward;
	float horizontal;
	float yaw;
	float max_linear_accel;
	float max_rad_accel;
	uint8_t enabled;
}chassis_control_t;

typedef struct
{
	int16_t projectile_speed;
	int16_t gun_feeding_speed;
	uint8_t override;
	uint8_t enabled;
}gun_control_t;

typedef struct
{
	uint8_t frame_header;
	int16_t y_pos;
	int16_t x_pos;
	float x_norm;
	float y_norm;
	uint8_t end_check;
	pid_data_t yaw_pid;
	pid_data_t pitch_pid;
	float x_offset;
	float y_offset;
	uint32_t last_time;
}xavier_packet_t;


#define SBC_GIMBAL_TURN_ANG_ID 0x11
#define SBC_GIMBAL_SET_ANG_ID 0x12
#define SBC_AIMBOT_NORM_ID 0x13


typedef __PACKED_STRUCT {
	float pitch;
	float yaw;
	uint8_t fire;
	int8_t spinspin;
	char padding[2];
}sbc_gimbal_data_t;


#define SBC_YOLO_BB_ID 0x21
typedef __PACKED_STRUCT {
	int16_t x_coord;
	int16_t y_coord;
	int16_t x_max;
	int16_t x_min;
	int16_t y_max;
	int16_t y_min;
}sbc_yolo_data_t;

typedef __PACKED_UNION {
	sbc_gimbal_data_t gimbal_data;
	sbc_yolo_data_t yolo_data;
}sbc_data_u;

typedef __PACKED_STRUCT {
    uint8_t header;
    uint8_t cmd_id; //set to 0x80
    uint8_t team;
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remaining_time;
    uint16_t ammo;
    uint8_t padding[5];
    uint8_t end_byte;
}sbc_game_data_t;

typedef __PACKED_STRUCT {
    uint8_t header;
    uint8_t cmd_id; //set to 0x80
    float pitch;
    float roll;
    float yaw;
    uint8_t end_byte;
}sbc_imu_data_t;

typedef struct {
	uint8_t cmd_id;
	sbc_data_u data;
}sbc_data_t;

typedef struct {
	uint8_t frame_header;
	sbc_data_t data;
	uint8_t frame_ender;
}sbc_raw_t;

typedef struct{
	uint8_t curr_gear;
	float spin_mult;
	float trans_mult;
	float accel_mult;
}speed_shift_t;


enum motor_params
{
	rpm_kp		= 1,
	rpm_ki		= 2,
	rpm_kd		= 3,
	angle_kp 	= 4,
	angle_ki 	= 5,
	angle_kd 	= 6,
	max_torque	= 7,
	center_angle= 8,
	max_angle	= 9,
	min_angle	= 10,
	max_rpm		=11,
};

enum motor_data
{
	motor_type	= 1,
	rpm			= 2,
	temp		= 3,
	angle 		= 4,
};

enum referee_data
{
	feeder_speed_limit = 1,
	projectile_speed_limit =2,
	chassis_power_limit = 3,
};

typedef enum {
	song,
	ok,
	not_ok,
	control_keyboard,
	control_control,
	control_sbc,
	bz_high,
	bz_low,
	bz_debug_low,
	bz_debug_high,
	bz_debug_rest,
}buzzing_type;


/*
enum aimbot_data
{
	//to be done
};

*/
#endif /* TASKS_INC_TYPEDEFS_H_ */
