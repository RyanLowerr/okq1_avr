
#include <avr/interrupt.h>

#include "controller.h"
#include "types.h"

CONTROLLER controller;
CONTROLLER controller_pre;

static volatile u08 controller_rxbuffer[CONTROLLER_BUFFER_SIZE];
static volatile u08 controller_rxhead;
static volatile u08 controller_rxtail;

ISR(USART1_RX_vect)
{
	u08 temp = UDR1;
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

	for(u08 i = 0; i < 8; i++)
		c->analog[i] = 512;
}

void controller_process(CONTROLLER *c)
{
	if(controller_buffer_size() >= 19)
		controller_read(&controller);
}

u08 controller_buffer_size(void)
{
	return (CONTROLLER_BUFFER_SIZE + controller_rxtail - controller_rxhead) % CONTROLLER_BUFFER_SIZE;
}

void controller_buffer_flush(void)
{
	for(u08 i = 0; i <= CONTROLLER_BUFFER_SIZE; i++)
		controller_rxbuffer[i] = 0;
	
	controller_rxhead = 0;
	controller_rxtail = 0;
}

u08 controller_buffer_read(void)
{
	u08 temp = controller_rxbuffer[controller_rxtail];
	controller_rxtail = (controller_rxtail + 1) % CONTROLLER_BUFFER_SIZE;
	return temp;
}

u08 controller_read(CONTROLLER *c)
{
	u08 high;
	u08 low;
	u16 analog[8];
	u16 checksum = 0;
	
	if(controller_buffer_read() == 0xff)
	{
		if(controller_buffer_read() == 0xff)
		{
			for(u08 i = 0; i < 8; i++)
			{
				high = controller_buffer_read();
				checksum += high;
				
				low = controller_buffer_read();
				checksum += low;
				
				analog[i] = (high << 8) + low;
				
				if(analog[i] > 1024) 
					analog[i] = 1024;
			}
			
			checksum = ~(checksum % 256);
			
			if((u08)checksum == controller_buffer_read())
			{
				for(u08 i = 0; i < 8; i++)
					c->analog[i] = analog[i];
			}
		}
	}
	
	controller_buffer_flush();
	
	return 1;
}

void controller_write(u08 c)
{
	while(bit_is_clear(UCSR1A, UDRE1));
	UDR1 = c;
}