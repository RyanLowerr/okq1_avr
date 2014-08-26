
#include <avr/interrupt.h>

#include "controller.h"
#include "types.h"

CONTROLLER controller;
CONTROLLER controller_pre;

static volatile u08 controller_txpacket[CONTROLLER_PACKET_SIZE];
static volatile u08 controller_rxpacket[CONTROLLER_PACKET_SIZE];
static volatile u08 controller_rxindex = 0;

static void controller_flush(void)
{
	for(u08 i = 0; i <= CONTROLLER_PACKET_SIZE; i++)
		controller_rxpacket[i] = 0;
	controller_rxindex = 0;
}

static u08 controller_read(void)
{
	u08 tmp;
	u16 analog[8];
	u08 n = 2;
	u16 checksum = 0;
	
	if((controller_rxpacket[0] != 0xff) && (controller_rxpacket[1] != 0xff))
		return 0;
	
	for(u08 i = 0; i < 4; i++)
	{
		tmp = controller_rxpacket[n++];
		checksum += tmp;
		analog[i] = tmp;
	}
	
	checksum = ~(checksum % 256);
	
	if((u08)checksum == controller_rxpacket[6])
	{
		for(u08 i = 0; i < 4; i++)
			controller.analog[i] = analog[i];
		return 1;
	}
	else
		return 0;
}

ISR(USART0_RX_vect)
{
	controller_rxpacket[controller_rxindex++] = UDR0;
	
	if(controller_rxindex >= 7)
	{
		controller_read();
		controller_flush();
	}
}

void controller_init(CONTROLLER *c)
{
	UBRR0H = ((F_CPU / 16 + CONTROLLER_BAUDRATE / 2) / CONTROLLER_BAUDRATE - 1) >> 8;
	UBRR0L = ((F_CPU / 16 + CONTROLLER_BAUDRATE / 2) / CONTROLLER_BAUDRATE - 1);
	
	UCSR0B |= (1 << TXEN0);
	UCSR0B |= (1 << RXEN0);
	UCSR0B |= (1 << RXCIE0);
	
	controller_flush();

	for(u08 i = 0; i < 8; i++)
		c->analog[i] = 0;
}

void controller_write(u08 c)
{
	while(bit_is_clear(UCSR0A, UDRE0));
	UDR0 = c;
}