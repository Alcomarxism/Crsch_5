#include "timeoutTimer.h"


void timeoutTimerInit(){
	TIMEOUTTIMERCONFB=(1<<CS22)|(0<<CS21)|(0<<CS20);
	TIMEOUTTIMERINTERCONF|=1<<TOV2;
	didTimeout=0;
	timoutTimerExp=0;
	TIMEOUTTIMER=0;
}
void startTimeout(uint8_t time){
	didTimeout=0;
	timoutTimerExp=time;
	TIMEOUTTIMER=0;
}

