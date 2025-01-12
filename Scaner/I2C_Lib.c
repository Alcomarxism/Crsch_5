#include "I2C_LIb.h"
#include "timeoutTimer.h"
#include <util/delay.h>
void I2C_Init()
{
	TWBR=0x20;
	I2C_err=0;
}

void I2C_Start()
{
	 TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	 startTimeout(I2CTIMEOUT);
	 while(!(TWCR&(1<<TWINT))){
		 if(didTimeout){
			 I2C_err=1;
			 return;
		 }
		 _delay_us(1);
	 }
}

void I2C_Stop()
{
	 TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void I2C_SendByte(uint8_t byte)
{
	TWDR = byte;
	TWCR = (1<<TWINT)|(1<<TWEN);
	startTimeout(I2CTIMEOUT);
	while (!(TWCR & (1<<TWINT))){
		if(didTimeout){
			I2C_err=1;
			return;
		}
		_delay_us(1);
	}
}

uint8_t I2C_ReciveByte()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	startTimeout(I2CTIMEOUT);
	while(!(TWCR & (1<<TWINT))){
		if(didTimeout){
			I2C_err=1;
			return 0xFFFF;
		}
		_delay_us(1);
	}
	return TWDR;
}

uint8_t I2C_ReciveByteLast()
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	startTimeout(I2CTIMEOUT);
	while(!(TWCR & (1<<TWINT)))	{
		if(didTimeout){
			I2C_err=1;
			return 0xFFFF;
		}
		_delay_us(1);
	}
	I2C_err=0;
	return TWDR;
}

void I2C_writeReg(uint8_t addr,uint8_t reg,uint8_t data){
	I2C_err=0;
	I2C_Start();
	I2C_SendByte(addr<<1);
	I2C_SendByte(reg);
	I2C_SendByte(data);
	I2C_Stop();
}

uint8_t I2C_readReg(uint8_t addr,uint8_t reg){
	I2C_err=0;
	I2C_Start();
	I2C_SendByte(addr<<1);
	I2C_SendByte(reg);
	I2C_Start();
	I2C_SendByte((addr<<1)+1);
	uint8_t bt=I2C_ReciveByteLast();
	I2C_Stop();
	return bt;
}

void I2C_writeReg16(uint8_t addr,uint8_t reg,uint16_t data){
	I2C_err=0;
	I2C_Start();
	I2C_SendByte(addr<<1);
	I2C_SendByte(reg);
	I2C_SendByte(data>>8);
	I2C_SendByte(data);
	I2C_Stop();
}

uint16_t I2C_readReg16(uint8_t addr,uint8_t reg){
	I2C_err=0;
	I2C_Start();
	I2C_SendByte(addr<<1);
	I2C_SendByte(reg);
	I2C_Start();
	I2C_SendByte((addr<<1)+1);
	uint16_t bt=((uint16_t)I2C_ReciveByte())<<8;
	bt|=(uint16_t)I2C_ReciveByteLast();
	I2C_Stop();
	return bt;
}
void I2C_writeReg32(uint8_t addr,uint8_t reg,uint32_t data){
	I2C_err=0;
	I2C_Start();
	I2C_SendByte(addr<<1);
	I2C_SendByte(reg);
	I2C_SendByte(data>>24);
	I2C_SendByte(data>>16);
	I2C_SendByte(data>>8);
	I2C_SendByte(data);
	I2C_Stop();
}

uint32_t I2C_readReg32(uint8_t addr,uint8_t reg){
	I2C_err=0;
	I2C_Start();
	I2C_SendByte(addr<<1);
	I2C_SendByte(reg);
	I2C_Start();
	I2C_SendByte((addr<<1)+1);
	uint32_t bt=((uint32_t)I2C_ReciveByte())<<24;
	bt|=((uint32_t)I2C_ReciveByte())<<16;
	bt|=((uint32_t)I2C_ReciveByte())<<8;
	bt|=(uint32_t)I2C_ReciveByteLast();
	I2C_Stop();
	return bt;
}

void I2C_writeBuff(uint8_t addr,uint8_t regStart,uint8_t *data,uint8_t cnt){
	I2C_err=0;
	I2C_Start();
	I2C_SendByte(addr<<1);
	I2C_SendByte(regStart);
	for(uint8_t i=0;i<cnt;i++)
		I2C_SendByte(data[i]);
	I2C_Stop();
}

void I2C_readToBuff(uint8_t addr,uint8_t regStart,uint8_t *data,uint8_t cnt){
	I2C_err=0;
	I2C_Start();
	I2C_SendByte(addr<<1);
	I2C_SendByte(regStart);
	I2C_Start();
	I2C_SendByte((addr<<1)+1);
	for(int i=0;i<(cnt-1);i++)
		data[i]=I2C_ReciveByte();
	data[cnt-1]=I2C_ReciveByteLast();
	I2C_Stop();
}