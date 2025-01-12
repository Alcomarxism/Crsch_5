#ifndef I2C_LIB_H_
#define I2C_LIB_H_
#include <avr/io.h>
#define I2CTIMEOUT 30

uint8_t I2C_err;
void I2C_Init();
void I2C_Start();
void I2C_Stop();
void I2C_SendByte(uint8_t byte);
uint8_t I2C_ReciveByte();
uint8_t I2C_ReciveByteLast();
void I2C_writeReg(uint8_t addr,uint8_t reg,uint8_t data);
uint8_t I2C_readReg(uint8_t addr,uint8_t reg);
void I2C_writeReg16(uint8_t addr,uint8_t reg,uint16_t data);
uint16_t I2C_readReg16(uint8_t addr,uint8_t reg);
void I2C_writeReg32(uint8_t addr,uint8_t reg,uint32_t data);
uint32_t I2C_readReg32(uint8_t addr,uint8_t reg);
void I2C_writeBuff(uint8_t addr,uint8_t regStart,uint8_t *data,uint8_t cnt);
void I2C_readToBuff(uint8_t addr,uint8_t regStart,uint8_t *data,uint8_t cnt);
#endif