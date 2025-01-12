#ifndef SERVO_LIB_H_
#define SERVO_LIB_H_
#include <avr/io.h>
void servoInit();
void setServoAngle(uint8_t ang,uint8_t servoNum);



#endif /* SERVO_LIB_H_ */