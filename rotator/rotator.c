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

struct flagglob
{
	unsigned azimuth	: 1; //
}Gflag;

char AzimuthDegree [4];

#define TX_RBUF_SIZE 256
uint8_t TxIdxIn;
volatile uint8_t TxIdxOut;
char TxUartBuffer [TX_RBUF_SIZE];

#define RX_RBUF_SIZE 256
volatile uint8_t RxIdxIn;
uint8_t RxIdxOut;
char RxUartBuffer [RX_RBUF_SIZE];

const char PROGMEM StrCRLF[] = "\r\n";
const char PROGMEM StrOk[] = "ok\r\n";
const char PROGMEM StrKey[] = "key ";
const char PROGMEM StrError[] = "\r\nerror: ";

const char PROGMEM StrError_azimuth_nvalid_range[] = "azimuth over 450";
const char PROGMEM StrError_bad_command[] = "bad command";
const char PROGMEM StrError_bad_digits[] = "must be 3 digits long";
PGM_P const PROGMEM ErrorTable[] =
{
	StrError_azimuth_nvalid_range,
	StrError_bad_command,
	StrError_bad_digits
};

const char PROGMEM StrNorth[] = "North";
const char PROGMEM StrEast[] = "East";
const char PROGMEM StrSouth[] = "South";
const char PROGMEM StrWest[] = "West";
PGM_P const PROGMEM DirectTable[] =
{
	StrNorth,
	StrEast,
	StrSouth,
	StrWest
};

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

void uart_send_error(enum Errors Error)
{
	uart_send_pstr(StrError);
	uart_send_pstr((PGM_P)pgm_read_word(&(ErrorTable[Error])));
	uart_send_pstr(StrCRLF);
};

void tick_2ms(void)
{
	static uint8_t Scaler;
	
	if (Scaler & 1)
	{
		uint8_t KeyCod;
		enum Directions Direct;
		
		KeyCod = get_key();
		if (KeyCod & 0x0F)
		{
			if (KeyCod & (1 << north)) Direct = north;
			if (KeyCod & (1 << east)) Direct = east;
			if (KeyCod & (1 << south)) Direct = south;
			if (KeyCod & (1 << west)) Direct = west;
			ant_switch(Direct);
			uart_send_pstr(StrKey);
			uart_send_pstr((PGM_P)pgm_read_word(&(DirectTable[Direct])));
			uart_send(' ');
			uart_send_pstr(StrOk);
		}
	}
	
	Scaler++;
}

uint8_t get_key(void)
{
	static uint8_t KeyCount[4];
	uint8_t KeyCod = 0;
	uint8_t Count;
	
	if (bit_is_clear(KEY_NORTH_PIN, KEY_NORTH)) KeyCod |= (1 << north);
	if (bit_is_clear(KEY_EAST_PIN, KEY_EAST)) KeyCod |= (1 << east);
	if (bit_is_clear(KEY_SOUTH_PIN, KEY_SOUTH)) KeyCod |= (1 << south);
	if (bit_is_clear(KEY_WEST_PIN, KEY_WEST)) KeyCod |= (1 << west);
	for (Count=0; Count<4; Count++)
	{
		if (KeyCod&(1<<Count))
		{
			if (KeyCount[Count])
			{
				KeyCod &= ~(1<<Count);
				if (KeyCount[Count] == KEY_HOLD_T)
				{
					KeyCod |= (16<<Count);
				}
				else
				{
					KeyCount[Count]++;
				}
			}
			else
			{
				KeyCount[Count]++;
			}
		}
		else
		{
			KeyCount[Count] = 0;
		}
	}
	return KeyCod;
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

enum Directions def_direction(uint16_t Angle)
{
	if (Angle > ANGLE_FULL_CIRCLE) Angle -= ANGLE_FULL_CIRCLE;
	if (Angle > ANGLE_WEST) return north;
	if (Angle > ANGLE_SOUTH) return west;
	if (Angle > ANGLE_EAST) return south;
	if (Angle > ANGLE_NORTH) return east;
	return north;
}

uint32_t str_to_num_ul(char * Str)
{
	uint8_t Digit, Count;
	uint32_t Num = 0;
	
	for (Count = 0; Count < 9; Count++)
	{
		Digit = (uint8_t)*Str++ - '0';
		if (Digit > 9) return Num;
		Num = Num * 10 + Digit;
	}
	return Num;
}

void azimuth_find(char RxByte)
{
	static uint8_t State = 0;
	static uint8_t StrIdx;
	
	if (!Gflag.azimuth)
	{
		if (State)
		{
			if (StrIdx == 3)
			{
				if (RxByte == '\r')
				{
					AzimuthDegree[StrIdx] = '\0';
					Gflag.azimuth = 1;
				}
				else uart_send_error(err_bad_digits);
				State = 0;
			} 
			else
			{
				if (((uint8_t)RxByte - '0') < 10U)
				{
					AzimuthDegree[StrIdx] = RxByte;
					StrIdx++;
				}
				else
				{
					State = 0;
					uart_send_error(err_bad_digits);
				}
			}
		}
		else
		{
			if ((RxByte == 'M') || (RxByte == 'm'))
			{
				State = 1;
				StrIdx = 0;
			}
			else uart_send_error(err_bad_command);
		}
	}
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
		if (RxIdxIn != RxIdxOut)
		{
			char RxByte = RxUartBuffer[RxIdxOut++];
			uart_send(RxByte);
			if (RxByte == '\r') uart_send('\n');
			azimuth_find(RxByte);
		}
		
		if (Gflag.azimuth)
		{
			uint16_t Azimuth;
			
			Azimuth = (uint16_t)str_to_num_ul(AzimuthDegree);
			if (Azimuth <= ANGLE_MAX)
			{
				ant_switch(def_direction(Azimuth));
				uart_send_pstr(StrOk);
			} 
			else
			{
				ClrBit(LED_PORT, LED);
				uart_send_error(err_azimuth_invalid_range);
			}
			Gflag.azimuth = 0;
		}
		
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