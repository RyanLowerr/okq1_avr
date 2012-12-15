
#ifndef _DYNAMIXEL_H_
#define _DYNAMIXEL_H_

#include <avr/io.h>

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
uint8_t dynamixel_ping(uint8_t id);
uint8_t dynamixel_readbyte(uint8_t id, uint8_t address, uint8_t* value);
uint8_t dynamixel_readword(uint8_t id, uint8_t address, uint16_t* value);
uint8_t dynamixel_readtable(uint8_t id, uint8_t start_address, uint8_t end_address, uint8_t* table);
uint8_t dynamixel_writebyte(uint8_t id, uint8_t address, uint8_t value);
uint8_t dynamixel_writeword(uint8_t id, uint8_t address, uint16_t value);
uint8_t dynamixel_syncwrite(uint8_t address, uint8_t length, uint8_t number, uint8_t* param);
uint8_t dynamixel_reset(uint8_t id);
uint16_t dynamixel_makeword(uint8_t lowbyte, uint8_t highbyte);
uint8_t dynamixel_getlowbyte(uint16_t word);
uint8_t dynamixel_gethighbyte(uint16_t word);

#endif
