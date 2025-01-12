#include "cheksum.h"
#include <util/crc16.h>
#include <avr/pgmspace.h>

uint16_t countCrc(){
	uint16_t crc = 0xFFFF;
	for (uint16_t addr = 0; addr <= FLASHEND; addr++) {
		uint8_t byte = pgm_read_byte_near(addr);
		crc = _crc16_update(crc, byte);
	}
	return crc;
}

uint16_t readCrcFromEEPROM(){
	EEAR = 0x00;
	EECR |= (1 << EERE);
	uint8_t high_byte = EEDR;
	EEAR = 0x01;
	EECR |= (1 << EERE);
	uint8_t low_byte = EEDR;
	return (uint16_t)(low_byte | (high_byte << 8));
}

void writeCrcToEEPROM(uint16_t crc){
	while (EECR & (1 << EEPE));
	EEAR = 0x00;
	EEDR = crc>>8;
	EECR |= (1 << EEMPE);
	EECR |= (1 << EEPE);
	while (EECR & (1 << EEPE));
	EEAR = 0x01;
	EEDR = crc;
	EECR |= (1 << EEMPE);
	EECR |= (1 << EEPE);
}