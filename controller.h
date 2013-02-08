
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "types.h"

#define CONTROLLER_BAUDRATE      38400
#define CONTROLLER_BUFFER_SIZE   128

// Controller states
#define CONTROLLER_LEG_INIT        0
#define CONTROLLER_LEG_STAND       1
#define CONTROLLER_LEG_SIT         2
#define CONTROLLER_LEG_WALK        4
#define CONTROLLER_LEG_DOWN        8
#define CONTROLLER_LEG_UP          16
#define CONTROLLER_LEG_TRACK0      32
#define CONTROLLER_LEG_TRACK1      64
#define CONTROLLER_LEG_TRACK2      128
#define CONTROLLER_LEG_TRACK3      256

#define CONTROLLER_TURRET_INIT     0
#define CONTROLLER_TURRET_HOME     1
#define CONTROLLER_TURRET_ABSOLUTE 2
#define CONTROLLER_TURRET_RELATIVE 4
#define CONTROLLER_TURRET_TRACK0   8
#define CONTROLLER_TURRET_TRACK1   16
#define CONTROLLER_TURRET_TRACK2   32
#define CONTROLLER_TURRET_TRACK3   64

#define CONTROLLER_GUN_INIT        0
#define CONTROLLER_GUN_HOME        1
#define CONTROLLER_GUN_ABSOLUTE    2
#define CONTROLLER_GUN_RELATIVE    4
#define CONTROLLER_GUN_TRACK0      8
#define CONTROLLER_GUN_TRACK1      16
#define CONTROLLER_GUN_TRACK2      32
#define CONTROLLER_GUN_TRACK3      64

typedef struct
{
	s16 x;
	s16 y;
	s16 z;
	s16 r;
	s16 s;
	u16 a[8];
	
	u16 leg_state;
	u16 turret_state;
	u16 gun_state;
} CONTROLLER;

extern CONTROLLER controller;

void controller_init(CONTROLLER *c);
void controller_process(CONTROLLER *C);
u08 controller_buffer_size(void);
void controller_buffer_flush(void);
u08 controller_buffer_read(void);
u08 controller_read(CONTROLLER *c);
void controller_write(u08 c);

#endif