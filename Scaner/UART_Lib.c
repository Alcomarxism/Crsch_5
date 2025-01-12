#include "UART_Lib.h"
#include "Global_defines.h"
#include <util/delay.h>
#define BAUD 9600L
#define MYUBRR (F_CPU/16/BAUD-1)

void UART_Init(){
	UCSR0A = 0;						
	UCSR0B= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00)| (0<<UCPOL0);
	UBRR0H=(uint8_t)MYUBRR>>8;
	UBRR0L=(uint8_t)MYUBRR;
}
void UART_write8(uint8_t data){
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 =(uint8_t)(data);
}

void UART_write16(uint16_t data){
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 =(uint8_t)(data>>8);
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 =(uint8_t)(data);
}

uint8_t UART_read8(){
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}
