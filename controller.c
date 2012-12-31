
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "controller.h"

CONTROLLER controller;

static volatile uint8_t controller_rxbuffer[CONTROLLER_BUFFER_SIZE];
static volatile uint8_t controller_rxhead;
static volatile uint8_t controller_rxtail;

ISR(USART1_RX_vect)
{
	uint8_t temp = UDR1;
	if(((controller_rxhead + 1) % CONTROLLER_BUFFER_SIZE) != controller_rxtail)
	{
		controller_rxbuffer[controller_rxhead] = temp;
		controller_rxhead = (controller_rxhead + 1) % CONTROLLER_BUFFER_SIZE; 
	}
}

void controller_init(CONTROLLER *c)
{
	UBRR1H = ((F_CPU / 16 + CONTROLLER_BAUDRATE / 2) / CONTROLLER_BAUDRATE - 1) >> 8;
	UBRR1L = ((F_CPU / 16 + CONTROLLER_BAUDRATE / 2) / CONTROLLER_BAUDRATE - 1);
	
	UCSR1B |= (1 << TXEN1);
	UCSR1B |= (1 << RXEN1);
	UCSR1B |= (1 << RXCIE1);
	
	controller_buffer_flush();

	c->x = 0.0;
	c->y = 0.0;
	c->z = 30.0;
	c->r = 0.0;
	c->s = 0.0;

	for(uint8_t i = 0; i < 8; i++)
		c->a[i] = 512;
}

uint8_t controller_buffer_size(void)
{
	return (CONTROLLER_BUFFER_SIZE + controller_rxtail - controller_rxhead) % CONTROLLER_BUFFER_SIZE;
}

void controller_buffer_flush(void)
{
	for(uint8_t i = 0; i <= CONTROLLER_BUFFER_SIZE; i++)
		controller_rxbuffer[i] = 0;
	
	controller_rxhead = 0;
	controller_rxtail = 0;
}

uint8_t controller_buffer_read(void)
{
	uint8_t temp = controller_rxbuffer[controller_rxtail];
	controller_rxtail = (controller_rxtail + 1) % CONTROLLER_BUFFER_SIZE;
	return temp;
}

uint8_t controller_read(CONTROLLER *c)
{
	uint8_t high;
	uint8_t low;
	uint16_t a[8];
	uint16_t checksum = 0;
	
	if(controller_buffer_read() == 0xff)
	{
		if(controller_buffer_read() == 0xff)
		{
			for(uint8_t i = 0; i < 8; i++)
			{
				high = controller_buffer_read();
				checksum += high;
				
				low = controller_buffer_read();
				checksum += low;
				
				a[i] = (high << 8) + low;
			}
			
			checksum = ~(checksum % 256);
			
			if((uint8_t)checksum == controller_buffer_read())
			{
				for(uint8_t i = 0; i < 8; i++)
					c->a[i] = a[i];
			}
		}
	}
	
	controller_buffer_flush();

	return 1;
}
