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

void *ProtocolRS::get_rs_data(rs_cmd_id_t cmd_id)
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

        default:
            return NULL;
            break;
    }
    return ptr;
}
