
#include <util/delay.h>
#include <avr/interrupt.h>

#include "dynamixel.h"
#include "types.h"

static volatile u08 dynamixel_txpacket[DYNAMIXEL_PACKET_SIZE];
static volatile u08 dynamixel_rxpacket[DYNAMIXEL_PACKET_SIZE];
static volatile u08 dynamixel_rxindex = 0;

ISR(USART_RX_vect)
{
	dynamixel_rxpacket[dynamixel_rxindex++] = UDR0;
}

void dynamixel_init(void)
{
	// Set UART baudrate
	UBRR0H = ((F_CPU / 16 + DYNAMIXEL_BAUDRATE / 2) / DYNAMIXEL_BAUDRATE - 1) >> 8;
	UBRR0L = ((F_CPU / 16 + DYNAMIXEL_BAUDRATE / 2) / DYNAMIXEL_BAUDRATE - 1);
	
	// Enable UART TX, RX, and RX interrupt
	UCSR0B |= (1 << TXEN0);
	UCSR0B |= (1 << RXEN0);
	UCSR0B |= (1 << RXCIE0);
	
	// Set UART direction pins as outputs
	DDRD |= (1 << PD2);
	DDRD |= (1 << PD3);
	
	// Reset rx index
	dynamixel_rxindex = 0;
}

static void dynamixel_settx(void)
{
	// Set UART direction pins
	PORTD |= (1 << PD2);
	PORTD &= ~(1 << PD3);
	
	//UCSR0B |= (1 << TXEN0);
	//UCSR0B &= ~(1 << RXEN0);
	//UCSR0B &= ~(1 << RXCIE0);
}

static void dynamixel_setrx(void)
{
	// Wait for TX complete flag before turning the bus around
	while(bit_is_clear(UCSR0A, TXC0));
	
	_delay_us(1);
	
	// Set UART direction pins
	PORTD &= ~(1 << PD2);
	PORTD |= (1 << PD3);
	
	//UCSR0B &= ~(1 << TXEN0);
	//UCSR0B |= (1 << RXEN0);
	//UCSR0B |= (1 << RXCIE0);
	
	// Reset rx index
	dynamixel_rxindex = 0;
}

static void dynamixel_write(u08 c)
{
	while(bit_is_clear(UCSR0A, UDRE0));
	UDR0 = c;
}

static u08 dynamixel_calculatechecksum(volatile u08 *packet)
{
	u16 checksum = 0;
	
	for(u08 i = DYNAMIXEL_ID; i <= (packet[DYNAMIXEL_LENGTH] + 2); i++)
		checksum += packet[i];
	
	return ~(checksum % 256);
}

static u08 dynamixel_writepacket(volatile u08 *txpacket, u08 packetlength)
{	
	for(u08 i = 0; i < packetlength; i++)
		dynamixel_write(txpacket[i]);
	
	return DYNAMIXEL_SUCCESS;
}

static u08 dynamixel_readpacket(volatile u08 *rxpacket, u08 packetlength)
{
	u16 ulcounter = 0;

	while(dynamixel_rxindex < packetlength)
	{
		if(ulcounter++ > 10000)
			return DYNAMIXEL_RX_TIMEOUT;
	}

	if((rxpacket[0] != 255) || (rxpacket[1] != 255))
		return DYNAMIXEL_RX_CORRUPT;
		
	if(rxpacket[packetlength - 1] != dynamixel_calculatechecksum(rxpacket))
		return DYNAMIXEL_RX_CORRUPT;

	return DYNAMIXEL_SUCCESS;
}

static u08 dynamixel_txrx(volatile u08 *txpacket, volatile u08 *rxpacket)
{
	u08 result;
	u08 rxlength = 0;
	u08 txlength = dynamixel_txpacket[DYNAMIXEL_LENGTH] + 4;
	
	txpacket[0] = (u08) 0xff;
	txpacket[1] = (u08) 0xff;
	txpacket[txlength - 1] = (u08) dynamixel_calculatechecksum(txpacket);
	
	dynamixel_settx();
	dynamixel_writepacket(txpacket, txlength);
	
	if(txpacket[DYNAMIXEL_ID] != DYNAMIXEL_BROADCAST_ID)
	{	
		if(txpacket[DYNAMIXEL_INSTRUCTION] == DYNAMIXEL_READ)
			rxlength = txpacket[DYNAMIXEL_PARAMETER + 1] + 6;
		else
			rxlength = 6;
			
		dynamixel_setrx();
		result = dynamixel_readpacket(rxpacket, rxlength);
				
		return result;			
	}
		
	return DYNAMIXEL_SUCCESS;
}

u08 dynamixel_ping(u08 id)
{	
	dynamixel_txpacket[DYNAMIXEL_ID]          = (u08) id;
	dynamixel_txpacket[DYNAMIXEL_LENGTH]      = (u08) 2;
	dynamixel_txpacket[DYNAMIXEL_INSTRUCTION] = (u08) DYNAMIXEL_PING;
	
	return dynamixel_txrx(dynamixel_txpacket, dynamixel_rxpacket);
}

u08 dynamixel_readbyte(u08 id, u08 address, u08 *value)
{
	u08 result;
	
	dynamixel_txpacket[DYNAMIXEL_ID]          = (u08) id;
	dynamixel_txpacket[DYNAMIXEL_LENGTH]      = (u08) 4;
	dynamixel_txpacket[DYNAMIXEL_INSTRUCTION] = (u08) DYNAMIXEL_READ;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER]   = (u08) address;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER+1] = (u08) 1;
	
	result = dynamixel_txrx(dynamixel_txpacket, dynamixel_rxpacket);
	
	if(result == DYNAMIXEL_SUCCESS)
		*value = dynamixel_rxpacket[DYNAMIXEL_PARAMETER];
	
	return result;
}

u08 dynamixel_readword(u08 id, u08 address, u16 *value)
{
	u08 result;
	
	dynamixel_txpacket[DYNAMIXEL_ID]          = (u08) id;
	dynamixel_txpacket[DYNAMIXEL_LENGTH]      = (u08) 4;
	dynamixel_txpacket[DYNAMIXEL_INSTRUCTION] = (u08) DYNAMIXEL_READ;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER]   = (u08) address;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER+1] = (u08) 2;
	
	result = dynamixel_txrx(dynamixel_txpacket, dynamixel_rxpacket);
	
	if(result == DYNAMIXEL_SUCCESS)
		*value = (u16) dynamixel_makeword(dynamixel_rxpacket[DYNAMIXEL_PARAMETER], dynamixel_rxpacket[DYNAMIXEL_PARAMETER+1]);
	
	return result;
}

u08 dynamixel_readtable(u08 id, u08 start_address, u08 end_address, u08 *table)
{
	u08 result;
	u08 length = end_address - start_address + 1;
	
	dynamixel_txpacket[DYNAMIXEL_ID]          = (u08) id;
	dynamixel_txpacket[DYNAMIXEL_LENGTH]      = (u08) 4;
	dynamixel_txpacket[DYNAMIXEL_INSTRUCTION] = (u08) DYNAMIXEL_READ;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER]   = (u08) start_address;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER+1] = (u08) length;
	
	result = dynamixel_txrx(dynamixel_txpacket, dynamixel_rxpacket);
	
	if(result == DYNAMIXEL_SUCCESS)
	{
		for(u08 i = 0; i < length; i++)
			table[start_address + i] = dynamixel_rxpacket[DYNAMIXEL_PARAMETER + i];
	}
	
	return result;
}

u08 dynamixel_writebyte(u08 id, u08 address, u08 value)
{	
	dynamixel_txpacket[DYNAMIXEL_ID]          = (u08) id;
	dynamixel_txpacket[DYNAMIXEL_LENGTH]      = (u08) 4;
	dynamixel_txpacket[DYNAMIXEL_INSTRUCTION] = (u08) DYNAMIXEL_WRITE;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER]   = (u08) address;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER+1] = (u08) value;
	
	return dynamixel_txrx(dynamixel_txpacket, dynamixel_rxpacket);
}

u08 dynamixel_writeword(u08 id, u08 address, u16 value)
{	
	dynamixel_txpacket[DYNAMIXEL_ID]          = (u08) id;
	dynamixel_txpacket[DYNAMIXEL_LENGTH]      = (u08) 5;
	dynamixel_txpacket[DYNAMIXEL_INSTRUCTION] = (u08) DYNAMIXEL_WRITE;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER]   = (u08) address;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER+1] = (u08) dynamixel_getlowbyte(value);
	dynamixel_txpacket[DYNAMIXEL_PARAMETER+2] = (u08) dynamixel_gethighbyte(value);
	
	return dynamixel_txrx(dynamixel_txpacket, dynamixel_rxpacket);
}

u08 dynamixel_syncwrite(u08 address, u08 length, u08 number, u08 *param)
{	
	dynamixel_txpacket[DYNAMIXEL_ID]          = (u08) DYNAMIXEL_BROADCAST_ID;
	dynamixel_txpacket[DYNAMIXEL_INSTRUCTION] = (u08) DYNAMIXEL_SYNC_WRITE;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER]   = (u08) address;
	dynamixel_txpacket[DYNAMIXEL_PARAMETER+1] = (u08) length;
	dynamixel_txpacket[DYNAMIXEL_LENGTH]      = (u08) ((length + 1) * number + 4);
	
	for(u08 i = 0; i < ((length + 1) * number); i++)
		dynamixel_txpacket[DYNAMIXEL_PARAMETER + 2 + i] = (u08) param[i];
	
	return dynamixel_txrx(dynamixel_txpacket, dynamixel_rxpacket);
}

u08 dynamixel_reset(u08 id)
{
	dynamixel_txpacket[DYNAMIXEL_ID]          = (u08) id;
	dynamixel_txpacket[DYNAMIXEL_LENGTH]      = (u08) 2;
	dynamixel_txpacket[DYNAMIXEL_INSTRUCTION] = (u08) DYNAMIXEL_RESET;
	
	return dynamixel_txrx(dynamixel_txpacket, dynamixel_rxpacket);
}

u16 dynamixel_makeword(u08 lowbyte, u08 highbyte)
{
	return ((highbyte << 8) + lowbyte);
}

u08 dynamixel_getlowbyte(u16 word)
{
	return (word & 0xff);
}

u08 dynamixel_gethighbyte(u16 word)
{
	return ((word & 0xff00) >> 8);
}