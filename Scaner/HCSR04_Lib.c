#include "HCSR04_Lib.h"
#include "UART_Lib.h"
void HCSR04_Init(){
	HCSR04_TRIG_PORTCONF|=1<<HCSR04_TRIG_PIN_N;
	HCSR04_ECHO_PORTCONF&=~(1<<HCSR04_ECHO_PIN_N);
	HCSR04_ECHO_PORT&=~(1<<HCSR04_ECHO_PIN_N);
	HCSR04_TIMERCONF=(1<<CS00)|(1<<CS02);
}
uint16_t HCSR04_Read(){
	HCSR04_TRIG_PORT|=1<<HCSR04_TRIG_PIN_N;
	_delay_us(TRTIME);
	HCSR04_TRIG_PORT&=~(1<<HCSR04_TRIG_PIN_N);
	startTimeout(SONICTIMEOUT);
	while(!(HCSR04_ECHO_PIN&(1<<HCSR04_ECHO_PIN_N))){
		if(didTimeout){
			return 0xFFFF;
		}
		_delay_us(1);
	}
	HCSR04_TIMER=0;
	startTimeout(SONICTIMEOUT);
	while(HCSR04_ECHO_PIN&(1<<HCSR04_ECHO_PIN_N)){
		if(didTimeout){
			return 0xFFFF;
		}
		_delay_us(1);
	}
	return DISTSTEP*HCSR04_TIMER;
}