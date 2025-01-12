#include "Servo_Lib.h"

#define LOWPWMSERVO 2000
#define  HIGHPWMSERVO 4000
#define  FREQPWMSERVO 40000
#define RANGEPWMSERVO (HIGHPWMSERVO-LOWPWMSERVO)
#define MIDPWMSERVO (LOWPWMSERVO+RANGEPWMSERVO/2)


void servoInit(){
	TCCR1A = (1<<COM1A1)|(1<<COM1B1) | (1<<WGM11);
	TCCR1B = (1<<WGM12)  | (1<<WGM13) | (1<<CS11);
	DDRB|=(1<<1)|(1<<2);
	ICR1=FREQPWMSERVO;
	OCR1A=0;
	OCR1B=0;
}
void setServoAngle(uint8_t ang,uint8_t servoNum){
	uint16_t pwm=ang;
	pwm=pwm*200/18+LOWPWMSERVO;
	if(servoNum==0)
		OCR1A=pwm;
	else if(servoNum==1)
		OCR1B=pwm;
}