#include <avr/io.h>
#define F_CPU 4000000UL
#include <util/delay.h>
#include "scd41.h"

#define SCD41_ADDR 0x62
#define CMD_START_PERIODIC_MEAS  0x21B1
#define CMD_GET_DATA_READY       0xE4B8
#define CMD_READ_MEASUREMENT     0xEC05

uint16_t co2_ppm = 0;
float temperature_C = 0;
float humidity_percent = 0;

//***************************************************************************
// Function Name : "TWI0_SCD41_init"
// Date : 04/24/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function configures the use of I2C on the AVR128DB48 with the SCD41
// Revision History : Initial version
//**************************************************************************
void TWI0_SCD41_init(void)
{
	TWI0.MBAUD = 0x01; //100kHz 
	TWI0.MCTRLA = TWI_ENABLE_bm;
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

//***************************************************************************
// Function Name : "scd41_write_command"
// Date : 04/24/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// Writes commands to the SCD41 through the use of I2C on the AVR128DB48
// Revision History : Initial version
//**************************************************************************
static void scd41_write_command(uint16_t cmd) 
{
	uint8_t data[2] = {cmd >> 8, cmd & 0xFF};
	TWI0.MADDR = (SCD41_ADDR << 1);
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	if (TWI0.MSTATUS & TWI_RXACK_bm) return;
	TWI0.MDATA = data[0];
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MDATA = data[1];
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MCTRLB = TWI_MCMD_STOP_gc;
}

//***************************************************************************
// Function Name : "scd41_data_ready"
// Date : 04/24/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// Writes command to scd41 to start periodic measurements
// and humidity
// Revision History : Initial version
//***************************************************************************
void scd41_start_periodic_measurement(void) 
{
	scd41_write_command(CMD_START_PERIODIC_MEAS);
}

//***************************************************************************
// Function Name : "scd41_data_ready"
// Date : 04/24/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// Reads in sensor data of the SCD41 through the use of I2C on the
// AVR128DB48 and uses the data to calculate the co2_ppm, temperature,
// and humidity
// Revision History : Initial version
//***************************************************************************
bool scd41_data_ready(void) {
	uint8_t cmd[2] = {CMD_GET_DATA_READY >> 8, CMD_GET_DATA_READY & 0xFF};
	TWI0.MADDR = (SCD41_ADDR << 1);
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MDATA = cmd[0];
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MDATA = cmd[1];
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MCTRLB = TWI_MCMD_STOP_gc;
	_delay_ms(1);

	uint8_t status[3];
	TWI0.MADDR = (SCD41_ADDR << 1) | 0x01;
	for (uint8_t i = 0; i < 3; i++) {
		while (!(TWI0.MSTATUS & TWI_RIF_bm));
		status[i] = TWI0.MDATA;
		if (i == 2)
		TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;
		else
		TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
	}
	return (status[1] & 0x07) != 0;
}

//***************************************************************************
// Function Name : "scd41_read_measurement"
// Date : 04/24/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// Reads in sensor data of the SCD41 through the use of I2C on the
// AVR128DB48 and uses the data to calculate the co2_ppm, temperature,
// and humidity
// Revision History : Initial version
//**************************************************************************
void scd41_read_measurement(void) 
{
	uint8_t buffer[9];
	TWI0.MADDR = (SCD41_ADDR << 1) | 0x01;
	for (uint8_t i = 0; i < 9; i++) 
	{
		while (!(TWI0.MSTATUS & TWI_RIF_bm));
		buffer[i] = TWI0.MDATA;
		if (i == 8)
		TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;
		else
		TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
	}

	co2_ppm = (buffer[0] << 8) | buffer[1];
	uint16_t raw_temp = (buffer[3] << 8) | buffer[4];
	uint16_t raw_rh = (buffer[6] << 8) | buffer[7];

	temperature_C = -45 + 175 * (raw_temp / 65535.0);
	humidity_percent = 100 * (raw_rh / 65535.0);
}
