
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "types.h"

#define CONTROLLER_BAUDRATE      38400
#define CONTROLLER_PACKET_SIZE   128

typedef struct
{
	u16 analog[8];
	u08 digital[3];
} CONTROLLER;

extern CONTROLLER controller;
extern CONTROLLER controller_pre;

void controller_init(CONTROLLER *c);
void controller_write(u08 c);

#endif