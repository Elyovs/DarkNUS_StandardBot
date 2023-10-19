/*
 * referee_msgs.h
 *
 *  Created on: Feb 21, 2023
 *      Author: cwx
 */

#ifndef BSP_INC_REFEREE_MSGS_H_
#define BSP_INC_REFEREE_MSGS_H_


#define REFEREE_DATA_SIZE (9 + INTERACTION_DATA_LEN)

#define REF_GAME_STATE_CMD_ID 0x0001

#pragma pack (1)

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;


typedef struct __packed//0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ref_game_state_t;


typedef enum
{
	NONE_WIN		= 0,
	RED_WIN			= 1,
	BLUE_WIN		= 2,
} ref_game_winner_t;


#define REF_GAME_RESULT_ID 0x0002
typedef struct __packed //0002
{
    uint8_t winner;
} ref_game_result_t;


#define REF_ROBOT_HP_CMD_ID 0x0003
typedef struct __packed //0x0003
{
    uint16_t red_1_HP;
    uint16_t red_2_HP;
    uint16_t red_3_HP;
    uint16_t red_4_HP;
    uint16_t red_5_HP;
    uint16_t red_7_HP;
    uint16_t red_base_HP;
    uint16_t blu_1_HP;
    uint16_t blu_2_HP;
    uint16_t blu_3_HP;
    uint16_t blu_4_HP;
    uint16_t blu_5_HP;
    uint16_t blu_7_HP;
    uint16_t blu_base_HP;
} ref_game_robot_HP_t;


#define REF_DART_LAUNCH_STATUS_CMD_ID 0x0004
typedef struct __packed //0x0004
{
	uint8_t dart_team;
	uint16_t dart_time;
} ref_dart_status_t;



#define REF_GAME_EVENT_CMD_ID 0x0101
typedef struct __packed //0101
{
    uint32_t event_type;
} ref_game_event_data_t;


#define REF_SUPPLIER_STATUS_CMD_ID 0x0102
typedef struct __packed //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ref_supply_projectile_data_t;


#define REF_SUPPLIER_BOOKING_CMD_ID 0x0103
typedef struct __packed //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ref_supply_projectile_booking_t;


#define REF_FOUL_CMD_ID 0x0104
typedef struct __packed //0x0104
{
    uint8_t level;
    uint8_t foul_robot_id;
} ref_referee_warning_t;


#define REF_DART_COOLDOWN_CMD_ID 0x0105
typedef struct __packed //0x0105
{
	uint8_t dart_cooldown;
} ref_dart_cooldown_t;


#define REF_ROBOT_DATA_CMD_ID 0x0201
typedef struct __packed //0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter17_heat0_cooling_rate;
    uint16_t shooter17_heat0_cooling_limit;
    uint16_t shooter17_heat0_speed_limit;
    uint16_t shooter17_heat1_cooling_rate;
    uint16_t shooter17_heat1_cooling_limit;
    uint16_t shooter17_heat1_speed_limit;
    uint16_t shooter42_heat1_cooling_rate;
    uint16_t shooter42_heat1_cooling_limit;
    uint16_t shooter42_heat1_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ref_game_robot_data_t;

typedef struct __packed //0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter17_heat0_cooling_rate;
    uint16_t shooter17_heat0_cooling_limit;
    uint16_t shooter17_heat1_cooling_rate;
    uint16_t shooter17_heat1_cooling_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ref_game_robot_data2_t;


#define REF_ROBOT_POWER_DATA_CMD_ID 0x0202
typedef struct __packed //0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
} ref_robot_power_data_t;


#define REF_ROBOT_POS_DATA_CMD_ID 0x0203
typedef struct __packed //0x0203
{
    float x;
    float y;
    float z;
    float yaw;
} ref_game_robot_pos_t;


#define REF_ROBOT_BUFF_DATA_CMD_ID 0x0204
typedef struct __packed //0x0204
{
    uint8_t power_rune_buff;
} ref_buff_data_t;


#define REF_AERIAL_ENERGY_DATA_CMD_ID 0x0205
typedef struct __packed //0x0205
{
    uint8_t attack_time;
} ref_aerial_robot_energy_t;


#define REF_ROBOT_DMG_DATA_CMD_ID 0x0206
typedef struct __packed //0x0206
{
    uint8_t armor_type : 4;
    uint8_t dmg_type : 4;
} ref_robot_dmg_t;


#define REF_ROBOT_SHOOT_DATA_CMD_ID 0x0207
typedef struct __packed //0x0207
{
	uint8_t shooter_id;
    uint8_t bullet_type;
    uint8_t bullet_freq;
    float bullet_speed;
} ref_shoot_data_t;


#define REF_ROBOT_MAGAZINE_DATA_CMD_ID 0x0208
typedef struct __packed //0x0208
{
    uint16_t magazine_17mm;
    uint16_t magazine_42mm;
    uint16_t game_coins;
} ref_magazine_data_t;

#define REF_RFID_BASE_ZONE 			(1)
#define REF_RFID_ELEVATED_ZONE 		(1<<1)
#define REF_RFID_POWER_RUNE_ZONE 	(1<<2)
#define REF_RFID_LAUNCH_RAMP_ZONE 	(1<<3)
#define REF_RFID_OUTPOST_ZONE 		(1<<4)
#define REF_RFID_REGEN_ZONE			(1<<6)
#define REF_RFID_ENGI_REGEN			(1<<7)

#define REF_ROBOT_RFID_BUFF_DATA_CMD_ID 0x0209
typedef struct __packed //0x209
{
	uint32_t rfid_buff;
} ref_rfid_status_t;


#define REF_DART_STATUS_CMD_ID 0x020A
typedef struct __packed //0x20A
{
	uint8_t dart_launcher_status;
	uint8_t dart_target_id;
	uint16_t target_change_time;
	uint8_t dart1_speed;
	uint8_t dart2_speed;
	uint8_t dart3_speed;
	uint8_t dart4_speed;
	uint16_t last_dart_launch_time;
	uint16_t last_dart_cmd_time;
} ref_dart_cmd_t;


#define REF_ROBOT_COMMS_CMD_ID 0x0301
typedef struct __packed//0x0301
{
	uint16_t cmd_ID;
    uint16_t send_ID;
    uint16_t receiver_ID;
} ref_inter_robot_data_t;

typedef struct __packed//0x0301
{
	uint16_t cmd_ID;
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint8_t graphic_operation;
    uint8_t graphic_layer;
} ref_delete_graphic_t;


#define REF_CUSTOM_DATA_CMD_ID 0x0302
typedef struct __packed
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} ref_custom_data_t;

//find the cmd id
typedef struct __packed
{
    uint8_t data[64];
} ref_up_stream_data_t;

typedef struct __packed
{
    uint8_t data[32];
} ref_download_stream_data_t;


typedef struct __packed
{
	uint8_t operation_type;
	uint8_t layer;
} ext_client_custom_graphic_delete_t;

typedef struct __packed
{
uint8_t graphic_name[3];
uint32_t operation_type:3;
uint32_t graphic_type:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
} graphic_data_struct_t;

typedef __PACKED_STRUCT
{
graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;


typedef union
{
	ref_game_state_t game_state;
	ref_game_result_t game_result;
	ref_game_robot_HP_t robot_hp;
	ref_dart_status_t dart_status;
	ref_game_event_data_t game_event;
	ref_supply_projectile_data_t projectile_supply_state;
	ref_supply_projectile_booking_t projectile_supply_queue;
	ref_referee_warning_t referee_warning;
	ref_dart_cooldown_t dart_cooldown;
	ref_game_robot_data_t robot_state;
	ref_robot_power_data_t power_data;
	ref_game_robot_pos_t robot_pos;
	ref_buff_data_t robot_buff;
	ref_aerial_robot_energy_t aerial_energy;
	ref_robot_dmg_t damage_data;
	ref_shoot_data_t shooting_data;
	ref_magazine_data_t magazine_data;
	ref_rfid_status_t rfid_buff;
	ref_dart_cmd_t dart_cmd;
	ref_inter_robot_data_t robot_comms;
	ref_custom_data_t custom_data;
	ref_up_stream_data_t upstream_data;
	ref_download_stream_data_t download_data;
} ref_data_u;

#define REF_HEADER_SIZE 5
typedef struct __packed
{
	uint8_t start_frame;
	uint16_t data_length;
	uint8_t seq;
	uint8_t crc;
	uint16_t cmd_id;
} ref_frame_header_t;

typedef struct __packed
{
	uint16_t cmd_id;
	ref_data_u data;
} ref_queue_t;


typedef struct __packed
{
	ref_frame_header_t start_header;
	ref_data_u data;
	uint16_t end_crc;
} ref_data_t;

typedef struct __packed
{
	uint16_t cmd_id;
	ref_data_u data;
} ref_msg_t;


#endif /* BSP_INC_REFEREE_MSGS_H_ */
