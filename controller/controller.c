
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "controller.h"

static volatile uint8_t controller_rxbuffer[CONTROLLER_BUFFER_SIZE];
static volatile uint8_t controller_rxhead = 0;
static volatile uint8_t controller_rxtail = 0;
uint8_t controller_paramaters[CONTROLLER_NUM_PARAMS];

ISR(USART1_RX_vect)
{
	uint8_t temp = UDR1;
	if(((controller_rxhead + 1) % CONTROLLER_BUFFER_SIZE) != controller_rxtail)
	{
		controller_rxbuffer[controller_rxhead] = temp;
		controller_rxhead = (controller_rxhead + 1) % CONTROLLER_BUFFER_SIZE; 
	}
}

void controller_init(void)
{
	UBRR1H = ((F_CPU / 16 + CONTROLLER_BAUDRATE / 2) / CONTROLLER_BAUDRATE - 1) >> 8;
	UBRR1L = ((F_CPU / 16 + CONTROLLER_BAUDRATE / 2) / CONTROLLER_BAUDRATE - 1);
	UCSR1B |= (1 << TXEN1);
	UCSR1B |= (1 << RXEN1);
	UCSR1B |= (1 << RXCIE1);
}

uint8_t controller_buffer_size(void)
{
	return (CONTROLLER_BUFFER_SIZE + controller_rxtail - controller_rxhead) % CONTROLLER_BUFFER_SIZE;
}

void controller_buffer_flush(void)
{
	controller_rxhead = controller_rxtail;
}

uint8_t controller_buffer_read(void)
{
	uint8_t temp = controller_rxbuffer[controller_rxtail];
	controller_rxtail = (controller_rxtail + 1) % CONTROLLER_BUFFER_SIZE;
	return temp;
}
