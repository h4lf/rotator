/*
 * rotator.h
 *
 * Created: 23.02.2016 19:38:45
 *  Author: UB4LAG Vasily Afanasyev
 */ 


#ifndef ROTATOR_H_
#define ROTATOR_H_

#define UART_UBRR 416 /* 1200 bps at 8MHz (0.2% error) */
#define TOP_TIMER2 249 /* 2mS at 8000000/64 */
#define KEY_HOLD_T 255U

#define ANGLE_MAX 450U
#define ANGLE_FULL_CIRCLE 360U
#define ANGLE_NORTH 45U /* 0-45 */
#define ANGLE_EAST ANGLE_NORTH + 90U
#define ANGLE_SOUTH ANGLE_EAST + 90U
#define ANGLE_WEST ANGLE_SOUTH + 90U

#define SetBit(port,bit) port |=  _BV(bit)
#define ClrBit(port,bit) port &= ~_BV(bit)
#define Port4Out(port,bit) port |=  _BV(bit)
#define Port4Inp(port,bit) port &= ~_BV(bit)
#define Bit(bit) _BV(bit)

#define LED B,5,H
#define R_NORTH D,2,H
#define R_EAST D,3,H
#define R_SOUTH D,4,H
#define R_WEST D,5,H
#define KEY_NORTH D,6,L
#define KEY_EAST D,7,L
#define KEY_SOUTH B,0,L
#define KEY_WEST B,1,L


typedef struct
{
	uint32_t	quot;
	uint8_t		rem;
}divmod10_t;

enum Directions {north, east, south, west, disconnect};

enum Errors {err_azimuth_invalid_range, err_bad_command, err_bad_digits};

void ioinit(void);
void uart_send(char);
void uart_send_hex_byte(uint8_t);
void uart_send_pstr(const char *);
void uart_send_error(enum Errors);
void tick_2ms(void);
uint8_t get_key(void);
void ant_switch(enum Directions);
enum Directions def_direction(uint16_t);
uint32_t str_to_num_ul(char *);
void azimuth_find(char);

#endif /* ROTATOR_H_ */