#include "protocol_rs.h"
#include "crc.h"
#include <stdlib.h>
#include <string.h>


ProtocolRS::ProtocolRS(const char *port_name) : Uart(port_name)
{
    if (fd != -1)
    {
		rs_thread_handle = std::thread(&ProtocolRS::rs_thread, this);
        printf("ProtocolRS thread started.\n");
    }
    return;
}

ProtocolRS::~ProtocolRS()
{
	if (rs_thread_handle.joinable()) 
	{
        rs_thread_handle.join();
    }
    return;
}

int ProtocolRS::rs_send(const uint16_t cmd_id, const uint8_t *data, const uint16_t len)
{
    send_header.sof = SOF;
    send_header.data_length = len;
    send_header.seq = 0;
    send_header.crc8 = 0;

    memset(send_buf, 0, sizeof(send_buf));

    Append_CRC8_Check_Sum((uint8_t *)&send_header, 5);
    memcpy(send_buf, &send_header, 5);
    memcpy(send_buf + 5, &cmd_id, 2);
    memcpy(send_buf + 7, data, len);
    Append_CRC16_Check_Sum(send_buf, len + 9);

    uart_write((const char*)send_buf, len + 9);

    return 0;
}

int ProtocolRS::rs_send_interaction(uint16_t data_cmd_id, uint16_t sender_id, uint16_t receiver_id, const uint8_t *user_data, const uint16_t len)
{
	robot_interaction_data_send = (robot_interaction_data_t *)malloc(len + 6);
	robot_interaction_data_send->data_cmd_id = data_cmd_id;
	robot_interaction_data_send->sender_id = sender_id;
	robot_interaction_data_send->receiver_id = receiver_id;
	memcpy(robot_interaction_data_send->user_data, user_data, len);
	rs_send(ROBOT_INTERACTION_DATA_ID, (uint8_t *)robot_interaction_data_send, len + 6);
	free(robot_interaction_data_send);
	return 0;
}

int ProtocolRS::rs_read_interaction(const robot_interaction_data_t *robot_interaction_data)
{
	switch (robot_interaction_data->data_cmd_id)
	{
	case INTERACTION_LAYER_DELETE_ID:
		memcpy(&interaction_layer_delete, robot_interaction_data->user_data, sizeof(interaction_layer_delete_t));
		printf("delete_type: %d, layer: %d\n", interaction_layer_delete.delete_type, interaction_layer_delete.layer);
		break;

	case INTERACTION_FIGURE_ID:
		memcpy(&interaction_figure, robot_interaction_data->user_data, sizeof(interaction_figure_t));
		printf("figure_name: %s, width: %d\n", interaction_figure.figure_name, interaction_figure.width);
		break;

	case INTERACTION_FIGURE_2_ID:
		memcpy(&interaction_figure_2, robot_interaction_data->user_data, sizeof(interaction_figure_2_t));
		break;

	case INTERACTION_FIGURE_3_ID:
		memcpy(&interaction_figure_3, robot_interaction_data->user_data, sizeof(interaction_figure_3_t));
		break;

	case INTERACTION_FIGURE_4_ID:
		memcpy(&interaction_figure_4, robot_interaction_data->user_data, sizeof(interaction_figure_4_t));
		break;

	case EXT_CLIENT_CUSTOM_CHARACTER_ID:
		memcpy(&ext_client_custom_character, robot_interaction_data->user_data, sizeof(ext_client_custom_character_t));
		break;

	case SENTRY_CMD_ID:
		memcpy(&sentry_cmd, robot_interaction_data->user_data, sizeof(sentry_cmd_t));
		break;

	case RADAR_CMD_ID:
		memcpy(&radar_cmd, robot_interaction_data->user_data, sizeof(radar_cmd_t));
		break;

	default:
		break;
	}
	return 0;
}

int ProtocolRS::rs_read(const uint8_t data)
{
	if (recv_index == 0)
	{
		if (data == SOF)
		{
			recv_buf[recv_index++] = data;
		}
	}
	else if (recv_index == 5)
	{
		if (Verify_CRC8_Check_Sum((uint8_t *)recv_buf, 5))
		{
			memcpy(&recv_header, recv_buf, 5);
			recv_buf[recv_index++] = data;
		}
		else
		{
			recv_index = 0;
			if (data == SOF)
			{
				recv_buf[recv_index++] = data;
			}
		}
	}
	else if (recv_index == recv_header.data_length + 9u)
	{
		if (Verify_CRC16_Check_Sum((uint8_t *)recv_buf, recv_header.data_length + 9))
		{
			uint16_t cmd_id;
			memcpy(&cmd_id, recv_buf + 5, 2);
			switch (cmd_id)
			{
                case GAME_STATUS_ID:
                    memcpy(&game_status, recv_buf + 7, recv_header.data_length);
                    break;

				case GAME_RESULT_ID:
					memcpy(&game_result, recv_buf + 7, recv_header.data_length);
					break;

				case GAME_ROBOT_HP_ID:
					memcpy(&game_robot_HP, recv_buf + 7, recv_header.data_length);
					break;

				case EVENT_DATA_ID:
					memcpy(&event_data, recv_buf + 7, recv_header.data_length);
					break;

				case EXT_SUPPLY_PROJECTILE_ACTION_ID:
					memcpy(&ext_supply_projectile_action, recv_buf + 7, recv_header.data_length);
					break;
				
				case REFEREE_WARNING_ID:
					memcpy(&referee_warning, recv_buf + 7, recv_header.data_length);
					break;
				
				case DART_INFO_ID:
					memcpy(&dart_info, recv_buf + 7, recv_header.data_length);
					break;

				case ROBOT_STATUS_ID:
					memcpy(&robot_status, recv_buf + 7, recv_header.data_length);
					break;

				case POWER_HEAT_DATA_ID:
					memcpy(&power_heat_data, recv_buf + 7, recv_header.data_length);
					break;

				case ROBOT_POS_ID:
					memcpy(&robot_pos, recv_buf + 7, recv_header.data_length);
					break;

				case BUFF_ID:
					memcpy(&buff, recv_buf + 7, recv_header.data_length);
					break;

				case AIR_SUPPORT_DATA_ID:
					memcpy(&air_support_data, recv_buf + 7, recv_header.data_length);
					break;

				case HURT_DATA_ID:
					memcpy(&hurt_data, recv_buf + 7, recv_header.data_length);
					break;

				case SHOOT_DATA_ID:
					memcpy(&shoot_data, recv_buf + 7, recv_header.data_length);
					break;

				case PROJECTILE_ALLOWANCE_ID:
					memcpy(&projectile_allowance, recv_buf + 7, recv_header.data_length);
					break;

				case RFID_STATUS_ID:
					memcpy(&rfid_status, recv_buf + 7, recv_header.data_length);
					break;

				case DART_CLIENT_CMD_ID:
					memcpy(&dart_client_cmd, recv_buf + 7, recv_header.data_length);
					break;

				case GROUND_ROBOT_POSITION_ID:
					memcpy(&ground_robot_position, recv_buf + 7, recv_header.data_length);
					break;

				case RADAR_MARK_DATA_ID:
					memcpy(&radar_mark_data, recv_buf + 7, recv_header.data_length);
					break;

				case SENTRY_INFO_ID:
					memcpy(&sentry_info, recv_buf + 7, recv_header.data_length);
					break;

				case RADAR_INFO_ID:
					memcpy(&radar_info, recv_buf + 7, recv_header.data_length);
					break;

				case ROBOT_INTERACTION_DATA_ID:
					if (robot_interaction_data_recv != NULL)
					{
						free(robot_interaction_data_recv);
						robot_interaction_data_recv = NULL;
					}
					robot_interaction_data_recv = (robot_interaction_data_t *)malloc(recv_header.data_length);
					memcpy(robot_interaction_data_recv, recv_buf + 7, recv_header.data_length);
					rs_read_interaction(robot_interaction_data_recv);
					break;

				case MAP_COMMAND_ID:
					memcpy(&map_command, recv_buf + 7, recv_header.data_length);
					break;

				case MAP_ROBOT_DATA_ID:
					memcpy(&map_robot_data, recv_buf + 7, recv_header.data_length);
					break;

				case MAP_DATA_ID:
					memcpy(&map_data, recv_buf + 7, recv_header.data_length);
					break;

				case CUSTOM_INFO_ID:
					memcpy(&custom_info, recv_buf + 7, recv_header.data_length);
					break;

				case CUSTOM_ROBOT_DATA_ID:
					memcpy(&custom_robot_data, recv_buf + 7, recv_header.data_length);
					break;

				case REMOTE_CONTROL_ID:
					memcpy(&remote_control, recv_buf + 7, recv_header.data_length);
					break;

				case CUSTOM_CLIENT_DATA_ID:
					memcpy(&custom_client_data, recv_buf + 7, recv_header.data_length);
					break;

				case MSG_C2AGX_ID:
					memcpy(&msg_c2agx, recv_buf + 7, recv_header.data_length);
					break;
				
				case MSG_AGX2C_ID:
					memcpy(&msg_agx2c, recv_buf + 7, recv_header.data_length);
					break;

                default:
                    break;
			}

			recv_index = 0;
			memset(recv_buf, 0, sizeof(recv_buf));
			memset(&recv_header, 0, sizeof(recv_header));

			if (data == SOF)
			{
				recv_buf[recv_index++] = data;
			}
			
			return 1;
		}
		else
		{
			recv_index = 0;
			if (data == SOF)
			{
				recv_buf[recv_index++] = data;
			}
		}
	}
	else
	{
		recv_buf[recv_index++] = data;
	}
	if (recv_index == 256)
	{
		recv_index = 0;
	}
	return 0;
}

void *ProtocolRS::rs_thread()
{
    while (1)
    {
        uint8_t data;
        uart_read((char*)&data, 1);
        rs_read(data);
    }
}

void *ProtocolRS::get_rs_data_interaction(rs_cmd_id_e cmd_id)
{
	void *ptr = NULL;
	switch (cmd_id)
	{
		case INTERACTION_LAYER_DELETE_ID:
			ptr = malloc(sizeof(interaction_layer_delete_t));
			memcpy(ptr, &interaction_layer_delete, sizeof(interaction_layer_delete_t));
			break;

		case INTERACTION_FIGURE_ID:
			ptr = malloc(sizeof(interaction_figure_t));
			memcpy(ptr, &interaction_figure, sizeof(interaction_figure_t));
			break;

		case INTERACTION_FIGURE_2_ID:
			ptr = malloc(sizeof(interaction_figure_2_t));
			memcpy(ptr, &interaction_figure_2, sizeof(interaction_figure_2_t));
			break;

		case INTERACTION_FIGURE_3_ID:
			ptr = malloc(sizeof(interaction_figure_3_t));
			memcpy(ptr, &interaction_figure_3, sizeof(interaction_figure_3_t));
			break;

		case INTERACTION_FIGURE_4_ID:
			ptr = malloc(sizeof(interaction_figure_4_t));
			memcpy(ptr, &interaction_figure_4, sizeof(interaction_figure_4_t));
			break;

		case EXT_CLIENT_CUSTOM_CHARACTER_ID:
			ptr = malloc(sizeof(ext_client_custom_character_t));
			memcpy(ptr, &ext_client_custom_character, sizeof(ext_client_custom_character_t));
			break;

		case SENTRY_CMD_ID:
			ptr = malloc(sizeof(sentry_cmd_t));
			memcpy(ptr, &sentry_cmd, sizeof(sentry_cmd_t));
			break;

		case RADAR_CMD_ID:
			ptr = malloc(sizeof(radar_cmd_t));
			memcpy(ptr, &radar_cmd, sizeof(radar_cmd_t));
			break;

		default:
			return NULL;
			break;
	}
	return ptr;
}

void *ProtocolRS::get_rs_data(rs_cmd_id_e cmd_id)
{
    void *ptr = NULL;
    switch (cmd_id)
    {
        case GAME_STATUS_ID:
            ptr = malloc(sizeof(game_status_t));
            memcpy(ptr, &game_status, sizeof(game_status_t));
            break;

		case GAME_RESULT_ID:
			ptr = malloc(sizeof(game_result_t));
			memcpy(ptr, &game_result, sizeof(game_result_t));
			break;

		case GAME_ROBOT_HP_ID:
			ptr = malloc(sizeof(game_robot_HP_t));
			memcpy(ptr, &game_robot_HP, sizeof(game_robot_HP_t));
			break;

		case EVENT_DATA_ID:
			ptr = malloc(sizeof(event_data_t));
			memcpy(ptr, &event_data, sizeof(event_data_t));
			break;

		case EXT_SUPPLY_PROJECTILE_ACTION_ID:
			ptr = malloc(sizeof(ext_supply_projectile_action_t));
			memcpy(ptr, &ext_supply_projectile_action, sizeof(ext_supply_projectile_action_t));
			break;

		case REFEREE_WARNING_ID:
			ptr = malloc(sizeof(referee_warning_t));
			memcpy(ptr, &referee_warning, sizeof(referee_warning_t));
			break;

		case DART_INFO_ID:
			ptr = malloc(sizeof(dart_info_t));
			memcpy(ptr, &dart_info, sizeof(dart_info_t));
			break;

		case ROBOT_STATUS_ID:
			ptr = malloc(sizeof(robot_status_t));
			memcpy(ptr, &robot_status, sizeof(robot_status_t));
			break;

		case POWER_HEAT_DATA_ID:
			ptr = malloc(sizeof(power_heat_data_t));
			memcpy(ptr, &power_heat_data, sizeof(power_heat_data_t));
			break;

		case ROBOT_POS_ID:
			ptr = malloc(sizeof(robot_pos_t));
			memcpy(ptr, &robot_pos, sizeof(robot_pos_t));
			break;

		case BUFF_ID:
			ptr = malloc(sizeof(buff_t));
			memcpy(ptr, &buff, sizeof(buff_t));
			break;

		case AIR_SUPPORT_DATA_ID:
			ptr = malloc(sizeof(air_support_data_t));
			memcpy(ptr, &air_support_data, sizeof(air_support_data_t));
			break;

		case HURT_DATA_ID:
			ptr = malloc(sizeof(hurt_data_t));
			memcpy(ptr, &hurt_data, sizeof(hurt_data_t));
			break;

		case SHOOT_DATA_ID:
			ptr = malloc(sizeof(shoot_data_t));
			memcpy(ptr, &shoot_data, sizeof(shoot_data_t));
			break;

		case PROJECTILE_ALLOWANCE_ID:
			ptr = malloc(sizeof(projectile_allowance_t));
			memcpy(ptr, &projectile_allowance, sizeof(projectile_allowance_t));
			break;

		case RFID_STATUS_ID:
			ptr = malloc(sizeof(rfid_status_t));
			memcpy(ptr, &rfid_status, sizeof(rfid_status_t));
			break;

		case DART_CLIENT_CMD_ID:
			ptr = malloc(sizeof(dart_client_cmd_t));
			memcpy(ptr, &dart_client_cmd, sizeof(dart_client_cmd_t));
			break;

		case GROUND_ROBOT_POSITION_ID:
			ptr = malloc(sizeof(ground_robot_position_t));
			memcpy(ptr, &ground_robot_position, sizeof(ground_robot_position_t));
			break;

		case RADAR_MARK_DATA_ID:
			ptr = malloc(sizeof(radar_mark_data_t));
			memcpy(ptr, &radar_mark_data, sizeof(radar_mark_data_t));
			break;

		case SENTRY_INFO_ID:
			ptr = malloc(sizeof(sentry_info_t));
			memcpy(ptr, &sentry_info, sizeof(sentry_info_t));
			break;

		case RADAR_INFO_ID:
			ptr = malloc(sizeof(radar_info_t));
			memcpy(ptr, &radar_info, sizeof(radar_info_t));
			break;

		case MAP_COMMAND_ID:
			ptr = malloc(sizeof(map_command_t));
			memcpy(ptr, &map_command, sizeof(map_command_t));
			break;

		case MAP_ROBOT_DATA_ID:
			ptr = malloc(sizeof(map_robot_data_t));
			memcpy(ptr, &map_robot_data, sizeof(map_robot_data_t));
			break;

		case MAP_DATA_ID:
			ptr = malloc(sizeof(map_data_t));
			memcpy(ptr, &map_data, sizeof(map_data_t));
			break;

		case CUSTOM_INFO_ID:
			ptr = malloc(sizeof(custom_info_t));
			memcpy(ptr, &custom_info, sizeof(custom_info_t));
			break;

		case CUSTOM_ROBOT_DATA_ID:
			ptr = malloc(sizeof(custom_robot_data_t));
			memcpy(ptr, &custom_robot_data, sizeof(custom_robot_data_t));
			break;

		case REMOTE_CONTROL_ID:
			ptr = malloc(sizeof(remote_control_t));
			memcpy(ptr, &remote_control, sizeof(remote_control_t));
			break;

		case CUSTOM_CLIENT_DATA_ID:
			ptr = malloc(sizeof(custom_client_data_t));
			memcpy(ptr, &custom_client_data, sizeof(custom_client_data_t));
			break;

		case MSG_C2AGX_ID:
			ptr = malloc(sizeof(msg_c2agx_t));
			memcpy(ptr, &msg_c2agx, sizeof(msg_c2agx_t));
			break;

		case MSG_AGX2C_ID:
			ptr = malloc(sizeof(msg_agx2c_t));
			memcpy(ptr, &msg_agx2c, sizeof(msg_agx2c_t));
			break;

        default:
            return NULL;
            break;
    }
    return ptr;
}
