
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <avr/io.h>

#define CONTROLLER_BAUDRATE      115200
#define CONTROLLER_BUFFER_SIZE   128

#define PARAM_TRAVEL_X           0
#define PARAM_TRAVEL_Y           1
#define PARAM_TRAVEL_Z           2
#define PARAM_TRAVEL_R           3
#define PARAM_GAIT_MODE          4
#define PARAM_TURRET_PAN         5
#define PARAM_TURRET_TILT        6
#define PARAM_TURRET_MODE        7
#define PARAM_GUN_STATUS         8
#define PARAM_GUN_MODE           9

#define CONTROLLER_NUM_PARAMS    9

void controller_init(void);
uint8_t controller_buffer_size(void);
void controller_buffer_flush(void);
uint8_t controller_buffer_read(void);

#endif
