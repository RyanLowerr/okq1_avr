
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <avr/io.h>

#define CONTROLLER_BAUDRATE      38400
#define CONTROLLER_BUFFER_SIZE   128

typedef struct
{
	float x;
	float y;
	float z;
	float r;
	float s;
	uint16_t a[8];
} CONTROLLER;

extern CONTROLLER controller;

void controller_init(CONTROLLER *c);
uint8_t controller_buffer_size(void);
void controller_buffer_flush(void);
uint8_t controller_buffer_read(void);

#endif
