#include "Global_defines.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include  "cheksum.h"
#include "Servo_Lib.h"
#include "UART_Lib.h"
#include "I2C_Lib.h"
#include "VL53l0X_Lib.h"
#include "UartMessageCodes.h"
#include "AnalyseData.h"
#include "HCSR04_Lib.h"
 
 
uint8_t isWork;
uint8_t minAng;
uint8_t maxAng;
uint8_t step;
ISR(TIMER2_OVF_vect){
	if(timoutTimerExp>0){
		timoutTimerExp--;
	}
	else{
		didTimeout=1;
	}
}

uint8_t selfdiagnostic(){
	uint16_t crc=countCrc();
	uint16_t prevcrc=readCrcFromEEPROM();
	if(prevcrc==0xFFFF){
		writeCrcToEEPROM(crc);
	}
	if(crc!=prevcrc){
		UART_write8(UARTERRORSELFDIAGMEM);
		return 0;
	}
	uint16_t dist1=HCSR04_Read();
	//if(dist1==0xFFFF){
	//	UART_write8(UARTERRORSELFDIAGSONIC);
	//	return 0;
	//}
	VL53L0X_askRangeSingleMillimeters();
	VL53L0X_readRangeSingleMillimeters();
	if(I2C_err){
		UART_write8(UARTERRORSELFDIAGLASER);
		return 0;
	}
	UART_write8(UARTDEVICESTART);
	return 1;
}

ISR(USART_RX_vect){
	switch(UART_read8()){
		case UARTSTOPCOMMAND:
			isWork=0;
			break;
		case UARTSTARTCOMMAND:
			isWork=1;
			break;
		case UARTSTATEREQUEST:
			if(isWork)
			UART_write8(UARTSTATEWORKRESPONSE);
			else
			UART_write8(UARTSTATEWAITRESPONSE);
			break;
		case UARTSETMINANG:
			minAng=UART_read8();
			break;
		case UARTSETMAXANG:
			maxAng=UART_read8();
			break;
		case UARTSETSTEP:
			step=UART_read8();
			break;
		case UARTSELFDIAGNOSTIC:
			selfdiagnostic();
		default:
			break;
	}
}


int main(void)
{
	UART_Init();
	I2C_Init();
	servoInit();
	VL53L0X_init(1);
	HCSR04_Init();
	timeoutTimerInit();
	sei();
	
	isWork=selfdiagnostic();
	uint8_t currAng=1;
	uint8_t rtdir=0;
	minAng=0;
	maxAng=180;
	step=1;
    while (1) 
    {
		if(isWork){
			VL53L0X_askRangeSingleMillimeters();
			uint16_t dist1=HCSR04_Read();
			if(dist1==0xFFFF){
				UART_write8(UARTERRORSONICTIMEOUT);
			}else if(dist1>SONICMAXRANGE){
				UART_write8(UARTERRORSONICOUTOFRANGE);
				dist1=0xFFFF;
			}else{
				UART_write8(UARTSENDSONIC);
				UART_write16(dist1);
			}
			uint16_t dist2=VL53L0X_readRangeSingleMillimeters()/10;
			if(I2C_err){
				UART_write8(UARTERRORLASERTIMEOUT);
				VL53L0X_init(1);
			}else if(dist2>LASERMAXRANGE){
				UART_write8(UARTERRORLASEROUTOFRANGE);
				dist2=0xFFFF;
			}else{
				UART_write8(UARTSENDLASER);
				UART_write16(dist2);
			}
			if(dist1==0xFFFF&&dist2==0xFFFF){
				UART_write8(UARTERRORMIDOUTOFRANGE);
			}else{
				UART_write8(UARTSENDMID);
				UART_write16(getMidDist(dist1,dist2));
			}
			UART_write8(UARTSENDANGLE);
			UART_write8(currAng);
			if(rtdir){
				if(currAng-step>0)
					currAng-=step;
				else
					currAng=0;
			}else{
				if(currAng+step<180)
					currAng+=step;
				else
					currAng=180;
			}
			if(currAng>=maxAng)
				rtdir=1;
			if(currAng<=minAng)
				rtdir=0;
			setServoAngle(currAng,1);
		}
	}
}


