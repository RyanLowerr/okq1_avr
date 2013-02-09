
#ifndef _DYNAMIXEL_H_
#define _DYNAMIXEL_H_

#include "types.h"

#define DYNAMIXEL_BAUDRATE      1000000

#define DYNAMIXEL_OK            1
#define DYNAMIXEL_TIMEOUT       2

#define DYNAMIXEL_ID            2
#define DYNAMIXEL_LENGTH        3
#define DYNAMIXEL_INSTRUCTION   4
#define DYNAMIXEL_ERROR         4
#define DYNAMIXEL_PARAMETER     5

#define DYNAMIXEL_BROADCAST_ID  254
#define DYNAMIXEL_PACKET_SIZE   128

#define DYNAMIXEL_PING          1
#define DYNAMIXEL_READ          2
#define DYNAMIXEL_WRITE         3
#define DYNAMIXEL_REG_WRITE     4
#define DYNAMIXEL_ACTION        5
#define DYNAMIXEL_RESET         6
#define DYNAMIXEL_SYNC_WRITE    131

#define DYNAMIXEL_SUCCESS       1
#define DYNAMIXEL_RX_CORRUPT    2
#define DYNAMIXEL_RX_TIMEOUT    3
#define DYNAMIXEL_TX_FAIL       4
#define DYNAMIXEL_TX_TIMEOUT    5

void dynamixel_init(void);
u08 dynamixel_ping(u08 id);
u08 dynamixel_readbyte(u08 id, u08 address, u08 *value);
u08 dynamixel_readword(u08 id, u08 address, u16 *value);
u08 dynamixel_readtable(u08 id, u08 start_address, u08 end_address, u08 *table);
u08 dynamixel_writebyte(u08 id, u08 address, u08 value);
u08 dynamixel_writeword(u08 id, u08 address, u16 value);
u08 dynamixel_syncwrite(u08 address, u08 length, u08 number, u08 *param);
u08 dynamixel_reset(u08 id);
u16 dynamixel_makeword(u08 lowbyte, u08 highbyte);
u08 dynamixel_getlowbyte(u16 word);
u08 dynamixel_gethighbyte(u16 word);

#endif