
#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <avr/io.h>

#define CAMERA_SLA   0x58

typedef struct
{
	uint16_t x[4];
	uint8_t xmin[4];
	uint8_t xmax[4];
	
	uint16_t y[4];
	uint8_t ymin[4];
	uint8_t ymax[4];
	
	uint8_t s[4];
	uint8_t I[4];
} camera_data;

void camera_init(void);
void camera_process(camera_data* cam);

#endif
