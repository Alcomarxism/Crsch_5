#ifndef UART_LIB_H_
#define UART_LIB_H_
#include "avr/io.h"
#include "timeoutTimer.h"

void UART_Init();
void UART_write8(uint8_t data);
void UART_write16(uint16_t data);
uint8_t UART_read8();
#endif /* UART_LIB_H_ */