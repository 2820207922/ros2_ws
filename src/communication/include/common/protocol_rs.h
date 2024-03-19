#ifndef PROTOCOL_RS_H
#define PROTOCOL_RS_H

#define __packed __attribute__((packed))

#include <stdint.h>
#include <string.h>
#include <thread>
#include "uart.h"

#define SOF 0xA5

typedef enum
{
	GAME_STATUS_ID = 0x0001,
	GAME_RESULT_ID = 0x0002,
	GAME_ROBOT_HP_ID = 0x0003,

	EVENT_DATA_ID = 0x0101,
	EXT_SUPPLY_PROJECTILE_ACTION_ID = 0x0102,
	REFEREE_WARNING_ID = 0x0104,
	DART_INFO_ID = 0X0105,

	ROBOT_STATUS_ID = 0x0201,
	POWER_HEAT_DATA_ID = 0x0202,
	ROBOT_POS_ID = 0x0203,
	BUFF_ID = 0x0204,
	AIR_SUPPORT_DATA_ID = 0x0205,
	HURT_DATA_ID = 0x0206,
	SHOOT_DATA_ID = 0x0207,
	PROJECTILE_ALLOWANCE_ID = 0x0208,
	RFID_STATUS_ID = 0x0209,
	DART_CLIENT_CMD_ID = 0x020A,
	GROUND_ROBOT_POSITION_ID = 0x020B,
	RADAR_MARK_DATA_ID = 0x020C,
	SENTRY_INFO_ID = 0x020D,
	RADAR_INFO_ID = 0x020E,

	ROBOT_INTERACTION_DATA_ID = 0x0301,
	INTERACTION_LAYER_DELETE_ID = 0x0100,
	INTERACTION_FIGURE_ID = 0x0101,
	INTERACTION_FIGURE_2_ID = 0x0102,
	INTERACTION_FIGURE_3_ID = 0x0103,
	INTERACTION_FIGURE_4_ID = 0x0104,
	EXT_CLIENT_CUSTOM_CHARACTER_ID = 0x0110,
	SENTRY_CMD_ID = 0x0120,
	RADAR_CMD_ID = 0x0121,

	MAP_COMMAND_ID = 0x0303,
	MAP_ROBOT_DATA_ID = 0x0305,
	MAP_DATA_ID = 0x0307,
	CUSTOM_INFO_ID = 0x0308,

	CUSTOM_ROBOT_DATA_ID = 0x0302,
	REMOTE_CONTROL_ID = 0x0304,
	CUSTOM_CLIENT_DATA_ID = 0x0306,

	MSG_IMU_INFO_ID = 0x0401,
	MSG_MOVE_CMD_ID = 0x0402
} rs_cmd_id_e;

typedef struct __packed
{
	uint8_t sof;
	uint16_t data_length;
	uint8_t seq;
	uint8_t crc8;
} frame_header_t;

typedef struct __packed
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} game_status_t;

typedef  struct __packed
{
    uint8_t winner;
} game_result_t;

typedef struct __packed
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} game_robot_HP_t;

typedef struct __packed
{
	uint32_t event_data;
} event_data_t;

typedef struct __packed
{
	uint8_t reserved;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef struct __packed
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
} referee_warning_t;

typedef struct __packed
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} dart_info_t;

typedef struct __packed
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
} robot_status_t;

typedef struct __packed
{
	uint16_t chassis_voltage;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

typedef struct __packed
{
	float x;
	float y;
	float angle;
} robot_pos_t;

typedef struct __packed
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
} buff_t;

typedef struct __packed
{
	uint8_t airforce_status;
	uint8_t time_remain;
} air_support_data_t;

typedef struct __packed
{
	uint8_t armor_id : 4;
	uint8_t HP_deduction_reason : 4;
} hurt_data_t;

typedef struct __packed
{
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
} shoot_data_t;

typedef struct __packed
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
} projectile_allowance_t;

typedef struct __packed
{
	uint32_t rfid_status;
} rfid_status_t;

typedef struct __packed
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

typedef struct __packed
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float standard_5_x;
	float standard_5_y;
} ground_robot_position_t;

typedef struct __packed
{
	uint8_t mark_hero_progress;
	uint8_t mark_engineer_progress;
	uint8_t mark_standard_3_progress;
	uint8_t mark_standard_4_progress;
	uint8_t mark_standard_5_progress;
	uint8_t mark_sentry_progress;
} radar_mark_data_t;

typedef struct __packed
{
	uint32_t sentry_info;
} sentry_info_t;

typedef struct __packed
{
	uint8_t radar_info;
} radar_info_t;

typedef struct __packed
{
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[];
} robot_interaction_data_t;

typedef struct __packed
{
	uint8_t delete_type;
	uint8_t layer;
} interaction_layer_delete_t;

typedef struct __packed
{
	uint8_t figure_name[3];
	uint32_t operate_tpye : 3;
	uint32_t figure_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t details_a : 9;
	uint32_t details_b : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t details_c : 10;
	uint32_t details_d : 11;
	uint32_t details_e : 11;
} interaction_figure_t;

typedef struct __packed
{
	interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

typedef struct __packed
{
	interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

typedef struct __packed
{
	interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;

typedef struct __packed
{
	interaction_figure_t interaction_figure;
	uint8_t data[30];
} ext_client_custom_character_t;

typedef struct __packed
{
	uint32_t sentry_cmd;
} sentry_cmd_t;

typedef struct __packed
{
	uint8_t radar_cmd;
} radar_cmd_t;

typedef struct __packed
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
} map_command_t;

typedef struct __packed
{
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
} map_robot_data_t;

typedef struct __packed
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
} map_data_t;

typedef struct __packed
{
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;

typedef struct __packed
{
	uint8_t data[30];
} custom_robot_data_t;

typedef struct __packed
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} remote_control_t;

typedef struct __packed
{
	uint16_t key_value;
	uint16_t x_position : 12;
	uint16_t mouse_left : 4;
	uint16_t y_position : 12;
	uint16_t mouse_right : 4;
	uint16_t reserved;
} custom_client_data_t;

// miniPC msg
typedef struct __packed
{
	float quat[4];
	float yaw;
	float pitch;
	float roll;
} msg_imu_info_t;

typedef struct __packed
{
	float vx;
	float vy;
	float vw;
	uint8_t top_flag;
} msg_move_cmd_t;

class ProtocolRS: public Uart
{
    public:
        ProtocolRS(const char *port_name);
        ~ProtocolRS();
        int rs_send(const uint16_t cmd_id, const uint8_t *data, const uint16_t len);
        int rs_send_interaction(uint16_t data_cmd_id, uint16_t sender_id, uint16_t receiver_id, const uint8_t *user_data, const uint16_t len);
        int rs_read(const uint8_t data);
        int rs_read_interaction(const robot_interaction_data_t *robot_interaction_data);
        void *rs_thread();
        void *get_rs_data(rs_cmd_id_e cmd_id);
        void *get_rs_data_interaction(rs_cmd_id_e cmd_id);
		
        game_status_t game_status;
        game_result_t game_result;
        game_robot_HP_t game_robot_HP;

        event_data_t event_data;
        ext_supply_projectile_action_t ext_supply_projectile_action;
        referee_warning_t referee_warning;
        dart_info_t dart_info;

        robot_status_t robot_status;
        power_heat_data_t power_heat_data;
        robot_pos_t robot_pos;
        buff_t buff;
        air_support_data_t air_support_data;
        hurt_data_t hurt_data;
        shoot_data_t shoot_data;
        projectile_allowance_t projectile_allowance;
        rfid_status_t rfid_status;
        dart_client_cmd_t dart_client_cmd;
        ground_robot_position_t ground_robot_position;
        radar_mark_data_t radar_mark_data;
        sentry_info_t sentry_info;
        radar_info_t radar_info;

        robot_interaction_data_t *robot_interaction_data_send, *robot_interaction_data_recv;
        interaction_layer_delete_t interaction_layer_delete;
        interaction_figure_t interaction_figure;
        interaction_figure_2_t interaction_figure_2;
        interaction_figure_3_t interaction_figure_3;
        interaction_figure_4_t interaction_figure_4;
        ext_client_custom_character_t ext_client_custom_character;
        sentry_cmd_t sentry_cmd;
        radar_cmd_t radar_cmd;

        map_command_t map_command;
        map_robot_data_t map_robot_data;
        map_data_t map_data;
        custom_info_t custom_info;

        custom_robot_data_t custom_robot_data;
        remote_control_t remote_control;
        custom_client_data_t custom_client_data;

		msg_imu_info_t msg_imu_info;
		msg_move_cmd_t msg_move_cmd;

    private:
    
        std::thread rs_thread_handle;

        uint8_t send_buf[256];
        uint8_t recv_buf[256];
        uint32_t recv_index = 0;
        frame_header_t recv_header, send_header;
};

#endif // PROTOCOL_RS_H