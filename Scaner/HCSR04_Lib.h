#ifndef HCSR04_LIB_H_
#define HCSR04_LIB_H_
#include <avr/io.h>
#include "Global_defines.h"
#include <util/delay.h>

#define SONICTIMEOUT 100
#define SONICMAXRANGE 200
#define TRTIME 20
#define DISTSTEP 1.1f
#define HCSR04_TRIG_PIN_N 2
#define HCSR04_ECHO_PIN_N 3
#define HCSR04_TRIG_PORT PORTD
#define HCSR04_ECHO_PORT PORTD
#define HCSR04_ECHO_PIN PIND
#define HCSR04_TRIG_PORTCONF DDRD
#define HCSR04_ECHO_PORTCONF DDRD
#define HCSR04_TIMER TCNT0
#define HCSR04_TIMERCONF TCCR0B

void HCSR04_Init();
uint16_t HCSR04_Read();


#endif /* HCSR04_LIB_H_ */