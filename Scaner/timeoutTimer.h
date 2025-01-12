#ifndef TIMEOUTTIMER_H_
#define TIMEOUTTIMER_H_
#include <avr/io.h>

#define TIMEOUTTIMER TCNT2
#define TIMEOUTTIMERCONFA TCCR2A
#define TIMEOUTTIMERCONFB TCCR2B
#define TIMEOUTTIMERINTERCONF TIMSK2


uint8_t didTimeout;
uint16_t timoutTimerExp;


void timeoutTimerInit();
void startTimeout(uint8_t time);

#endif /* TIMEOUTTIMER_H_ */