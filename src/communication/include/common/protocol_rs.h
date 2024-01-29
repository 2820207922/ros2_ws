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
} rs_cmd_id_t;

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

class ProtocolRS: public Uart
{
    public:
        ProtocolRS(const char *port_name);
        ~ProtocolRS();
        int rs_send(const uint16_t cmd_id, const uint8_t *data, const uint16_t len);
        int rs_read(const uint8_t data);
        void *rs_thread();
        void *get_rs_data(rs_cmd_id_t cmd_id);

    private:
    
        std::thread rs_thread_handle;

        uint8_t send_buf[256];
        uint8_t recv_buf[256];
        uint32_t recv_index = 0;
        frame_header_t recv_header, send_header;
        
        game_status_t game_status;
        game_result_t game_result;
};

#endif // PROTOCOL_RS_H