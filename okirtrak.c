
#include <avr/io.h>
#include <util/delay.h>

#include "okirtrak.h"
#include "twi.h"

void okirtrak_init(OKIRTRAK *cam, uint8_t address)
{
	twi_init();
	
	cam->address = address;
	
	uint8_t tx[128];
	_delay_ms(100);
	
	tx[0] = 0x30; tx[1] = 0x01;
	twi_write(cam->address, &tx[0], 2);
	_delay_ms(100);
		
	tx[0] = 0x00; tx[1] = 0x00;	tx[2] = 0x00; tx[3] = 0x00; tx[4] = 0x00; tx[5] = 0x00; tx[6] = 0x00; tx[7] = 0x90;
	twi_write(cam->address, &tx[0], 8);
	_delay_ms(100);
	
	tx[0] = 0x07; tx[1] = 0x00; tx[2] = 0x41;
	twi_write(cam->address, &tx[0], 3);
	_delay_ms(100);
		
	tx[0] = 0x07; tx[1] = 0x00; tx[2] = 0x41;
	twi_write(cam->address, &tx[0], 3);
	_delay_ms(100);
		
	tx[0] = 0x1A; tx[1] = 0x40; tx[2] = 0x00;
	twi_write(cam->address, &tx[0], 3);
	_delay_ms(100);
		
	tx[0] = 0x33; tx[1] = 0x05;
	twi_write(cam->address, &tx[0], 2);
	_delay_ms(100);
	
	tx[0] = 0x30; tx[1] = 0x08;
	twi_write(cam->address, &tx[0], 2);
	_delay_ms(100);
}

void okirtrak_process(OKIRTRAK *cam)
{
	uint8_t tx[128];
	uint8_t rx[128];

	tx[0] = 0x37;
	twi_write(cam->address, &tx[0], 1);
	_delay_us(380);
	
	twi_read(cam->address, &rx[0], 36);
	_delay_us(380);
	
	for(uint8_t i = 0; i < 4; i++)
	{
		cam->x[i] = rx[i*9]   + ((rx[i*9+2] & 0x30) << 4);
		cam->y[i] = rx[i*9+1] + ((rx[i*9+2] & 0xc0) << 2);
		cam->s[i] = rx[i*9+2] & 0x0f;
		cam->xmin[i] = rx[i*9+3];
		cam->ymin[i] = rx[i*9+4]; 
		cam->xmax[i] = rx[i*9+5];
		cam->ymax[i] = rx[i*9+6];
		cam->I[i] = rx[i*9+8];
	}
}
