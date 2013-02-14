
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "types.h"

#define CONTROLLER_BAUDRATE      38400
#define CONTROLLER_BUFFER_SIZE   128

typedef struct
{
	u16 analog[8];
	u08 digital[3];
} CONTROLLER;

extern CONTROLLER controller;
extern CONTROLLER controller_pre;

void controller_init(CONTROLLER *c);
void controller_process(CONTROLLER *C);
u08 controller_buffer_size(void);
void controller_buffer_flush(void);
u08 controller_buffer_read(void);
u08 controller_read(CONTROLLER *c);
void controller_write(u08 c);

#endif