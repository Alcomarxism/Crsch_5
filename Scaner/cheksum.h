#ifndef CHEKSUM_H_
#define CHEKSUM_H_
#include <avr/io.h>
#define CRCEEPROMADDR 0x00;

uint16_t countCrc();
uint16_t readCrcFromEEPROM();
void writeCrcToEEPROM(uint16_t crc);


#endif /* CHEKSUM_H_ */