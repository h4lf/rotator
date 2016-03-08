/*
 * rotator.c
 *
 * GS-232A Computer Control Interface for Antenna Rotators
 * 
 * Created: 20.02.2016 20:54:31
 *  Author: UB4LAG Vasily Afanasyev

 The MIT License (MIT)

 Copyright (c) 2016 Vasily Afanasyev

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */ 


#define F_CPU  8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include "rotator.h"

volatile struct isrflagglob
{
	unsigned itmr2		: 1; // 
} IsrFlag;

#define TX_RBUF_SIZE 256
uint8_t TxIdxIn;
volatile uint8_t TxIdxOut;
char TxUartBuffer [TX_RBUF_SIZE];

#define RX_RBUF_SIZE 256
volatile uint8_t RxIdxIn;
uint8_t RxIdxOut;
char RxUartBuffer [RX_RBUF_SIZE];

void ioinit(void)
{
	//------InitCLK
	CLKPR = 1 << CLKPCE; // CLK Change Enable
	CLKPR = 1 << CLKPS0; // CLK/2
	
	//------
	ACSR = 0x80;
	
	//------InitGPIO
	Port4Out(R_NORTH_DDR, R_NORTH);
	Port4Out(R_EAST_DDR, R_EAST);
	Port4Out(R_SOUTH_DDR, R_SOUTH);
	Port4Out(R_WEST_DDR, R_WEST);
	ant_switch(north);
	
	Port4Inp(KEY_NORTH_DDR, KEY_NORTH);
	Port4Inp(KEY_EAST_DDR, KEY_EAST);
	Port4Inp(KEY_SOUTH_DDR, KEY_SOUTH);
	Port4Inp(KEY_WEST_DDR, KEY_WEST);
	SetBit(KEY_NORTH_PORT, KEY_NORTH);
	SetBit(KEY_EAST_PORT, KEY_EAST);
	SetBit(KEY_SOUTH_PORT, KEY_SOUTH);
	SetBit(KEY_WEST_PORT, KEY_WEST);
	
	Port4Out(LED_DDR, LED);
	SetBit(LED_PORT, LED);
	
	//------Init TIMER2
	TCCR2A = Bit(WGM21); // Clear Timer on Compare Match (CTC) mode
	OCR2A = TOP_TIMER2;
	TIMSK2 = Bit(OCIE2A);
	TCCR2B = Bit(CS22); // clkT2S/64
	
	//------Init USART
	UBRR0 = UART_UBRR;
	UCSR0B = (	1 << TXEN0 |
				1 << RXEN0 );
	UCSR0B |= (	0 << UDRIE0 |
				1 << RXCIE0 );
	sei();
}

void uart_send(char Byte)
{
	TxUartBuffer[TxIdxIn++] = Byte;
}

void uart_send_hex_byte(uint8_t Byte)
{
	uint8_t TmpChar;
	TmpChar = ((Byte >> 4) & 0x0f) + 0x30;
	uart_send((TmpChar < 0x3a) ? TmpChar : (TmpChar+7));
	TmpChar = (Byte & 0x0f) + 0x30;
	uart_send((TmpChar < 0x3a) ? TmpChar : (TmpChar+7));
}

void uart_send_pstr(const char *Str)
{
	char Byte;
	while((Byte = pgm_read_byte(Str++)) != '\0')
	{
		uart_send(Byte); // Points to one ASCII to be written one at a time.
	}
}

void tick_2ms(void)
{
	
}

void ant_switch(enum Directions Direct)
{
	ClrBit(R_NORTH_PORT, R_NORTH);
	ClrBit(R_EAST_PORT, R_EAST);
	ClrBit(R_SOUTH_PORT, R_SOUTH);
	ClrBit(R_WEST_PORT, R_WEST);
	switch(Direct)
	{
		case north: SetBit(R_NORTH_PORT, R_NORTH);
		break;
		case east: SetBit(R_EAST_PORT, R_EAST);
		break;
		case south: SetBit(R_SOUTH_PORT, R_SOUTH);
		break;
		case west: SetBit(R_WEST_PORT, R_WEST);
		break;
		case disconnect:
		break;
	}
}

enum Directions def_direction(uint16_t angle)
{
	if (angle > ANGLE_FULL_CIRCLE) angle -= ANGLE_FULL_CIRCLE;
	if (angle > ANGLE_WEST) return north;
	if (angle > ANGLE_SOUTH) return west;
	if (angle > ANGLE_EAST) return south;
	if (angle > ANGLE_NORTH) return east;
	return north;
}

uint32_t str_to_num_ul(char * str)
{
	uint8_t digit, i;
	uint32_t num = 0;
	
	for (i = 0; i < 9; i++)
	{
		digit = (uint8_t)*str++ - '0';
		if (digit > 9) return num;
		num = num * 10 + digit;
	}
	return num;
}

ISR(TIMER2_COMPA_vect)
{
	IsrFlag.itmr2 = 1;
}

ISR(USART_RX_vect)
{
	RxUartBuffer[RxIdxIn++] = UDR0;
}

ISR(USART_UDRE_vect)
{
	if (TxIdxIn != TxIdxOut)
	{
		// данные есть
		UDR0 = TxUartBuffer[TxIdxOut++];
	}
	else
	{
		UCSR0B &= ~(1<<UDRIE0); // запретить прерывание "регистр данных пуст"
	}
}

int main(void)
{
	ioinit();
	
    while(1)
    {
		
		if (IsrFlag.itmr2)
		{
			cli();
			IsrFlag.itmr2 = 0;
			sei();
			tick_2ms();
		}
		
		if (bit_is_set(UCSR0A, UDRE0) && (TxIdxIn != TxIdxOut))
		{ // data register empty
				UCSR0B |= (1<<UDRIE0); // data register empty interrupt enable
		}
        //TODO:: Please write your application code 
    }
}