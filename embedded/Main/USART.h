#ifndef __USART_H__
#define __USART_H__

#include <avr/io.h>
#include <stdint.h>

#define BAUD_RATE 9600
#define USART1_BAUD_RATE(BAUD_RATE) ((float)(64*8000000/(16*(float)BAUD_RATE)+0.5))


void USART1_init(void);
void USART1_Transmit(unsigned char c);
unsigned char USART1_Receive(void);
void USART1_string_send(char* str);
void USART1_HEX_send(uint8_t);
void USART1_DEC_send(uint32_t);
void USART1_bytes_send(uint8_t*, uint8_t);
void USART1_Accel_send(uint8_t*);


#endif
