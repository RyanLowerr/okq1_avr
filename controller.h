
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "types.h"

#define CONTROLLER_BAUDRATE      38400
#define CONTROLLER_BUFFER_SIZE   128

typedef struct
{
	float x;
	float y;
	float z;
	float r;
	float s;
	u16 a[8];
} CONTROLLER;

extern CONTROLLER controller;

void controller_init(CONTROLLER *c);
u08 controller_buffer_size(void);
void controller_buffer_flush(void);
u08 controller_buffer_read(void);
u08 controller_read(CONTROLLER *c);

#endif
