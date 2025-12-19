#ifndef SERLCD_H
#define SERLCD_H

#include <stdint.h>

void SerLCD_init(void);
void update_SerLCD(uint16_t co2, float temp_C, float rh);

#endif
