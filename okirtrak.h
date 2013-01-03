
#ifndef _OKIRTRAK_H_
#define _OKIRTRAK_H_

#include <avr/io.h>

typedef struct
{
	uint8_t address;

	uint16_t x[4];
	uint8_t xmin[4];
	uint8_t xmax[4];
	
	uint16_t y[4];
	uint8_t ymin[4];
	uint8_t ymax[4];
	
	uint8_t s[4];
	uint8_t I[4];
} OKIRTRAK;

void okirtrak_init(OKIRTRAK *cam, uint8_t address);
void okirtrak_process(OKIRTRAK *cam);

#endif
