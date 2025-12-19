#include <avr/io.h>
#define F_CPU 4000000UL
#include <util/delay.h>
#include <stdio.h>
#include "serlcd.h"

#define SERLCD_ADDR 0x72
char dsp_buff1[21];
char dsp_buff2[21];
char dsp_buff3[21];
char dsp_buff4[21];
//***************************************************************************
// Function Name : "write_I2C_SerLCD"
// Date : 04/24/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function writes a character to the SerLCD via I2C on the AVR128DB48
// Revision History : Initial version
//**************************************************************************
static void write_I2C_SerLCD(unsigned char data) 
{
	TWI0.MADDR = (SERLCD_ADDR << 1);
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	if (TWI0.MSTATUS & TWI_RXACK_bm) {
		TWI0.MCTRLB = TWI_MCMD_STOP_gc;
		return;
	}
	TWI0.MDATA = data;
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}

//***************************************************************************
// Function Name : "write_string_to_SerLCD"
// Date : 04/24/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function writes a string to the SerLCD via I2C on the AVR128DB48
// Revision History : Initial version
//**************************************************************************
static void write_string_to_SerLCD(const char *str) {
	while (*str) {
		write_I2C_SerLCD(*str++);
		_delay_ms(2);
	}
}

//***************************************************************************
// Function Name : "SerLCD_init"
// Date : 04/24/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function clears the serlcd screen and sets the cursor to home
// Revision History : Initial version
//**************************************************************************
void SerLCD_init(void) {
	_delay_ms(500);
	write_string_to_SerLCD("|-");  // Clear + home
	_delay_ms(10);
}

//***************************************************************************
// Function Name : "update_SerLCD"
// Date : 04/24/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function stores strings in each display buffer which represents a
// line in the SerLCD and then prints the messages accordingly
// Revision History : Initial version
//**************************************************************************
void update_SerLCD(uint16_t co2, float temp_C, float rh) 
{
	write_string_to_SerLCD("|-");

	write_string_to_SerLCD(dsp_buff1);
	write_string_to_SerLCD(dsp_buff2);
	write_string_to_SerLCD(dsp_buff3);
	write_string_to_SerLCD(dsp_buff4);
}
