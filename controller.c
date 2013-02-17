
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
	u08 high;
	u08 low;
	u16 analog[8];
	u08 n = 2;
	u16 checksum = 0;
	
	if((controller_rxpacket[0] != 0xff) && (controller_rxpacket[1] != 0xff))
		return 0;
	
	for(u08 i = 0; i < 8; i++)
	{
		high = controller_rxpacket[n++];
		checksum += high;
		
		low = controller_rxpacket[n++];
		checksum += low;
		
		analog[i] = (high << 8) + low;
	}
	
	checksum = ~(checksum % 256);
	
	if((u08)checksum == controller_rxpacket[18])
	{
		for(u08 i = 0; i < 8; i++)
			controller.analog[i] = analog[i];
		return 1;
	}
	else
		return 0;
}

ISR(USART1_RX_vect)
{
	controller_rxpacket[controller_rxindex++] = UDR1;
	
	if(controller_rxindex >= 19)
	{
		controller_read();
		controller_flush();
	}
}

void controller_init(CONTROLLER *c)
{
	UBRR1H = ((F_CPU / 16 + CONTROLLER_BAUDRATE / 2) / CONTROLLER_BAUDRATE - 1) >> 8;
	UBRR1L = ((F_CPU / 16 + CONTROLLER_BAUDRATE / 2) / CONTROLLER_BAUDRATE - 1);
	
	UCSR1B |= (1 << TXEN1);
	UCSR1B |= (1 << RXEN1);
	UCSR1B |= (1 << RXCIE1);
	
	controller_flush();

	for(u08 i = 0; i < 8; i++)
		c->analog[i] = 512;
}

void controller_write(u08 c)
{
	while(bit_is_clear(UCSR1A, UDRE1));
	UDR1 = c;
}