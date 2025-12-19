#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"
#include <avr/io.h>
#define F_CPU 4000000
#include <util/delay.h>

int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx) //we only have 1 bus
{ 	
    /* TODO:IMPLEMENT or leave empty if all sensors are located on one single
     * bus
     */
    return NOT_IMPLEMENTED_ERROR;
}

void sensirion_i2c_hal_init(void) 
{
	TWI0.MBAUD = 0x01; 
	TWI0.MCTRLA = TWI_ENABLE_bm;
	TWI0.DBGCTRL = 0x01; 
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

/**
 * Release all resources initialized by sensirion_i2c_hal_init().
 */
void sensirion_i2c_hal_free(void) { //NO DYNAMIC MEMORY USAGE SO IGNORE
	
    /* TODO:IMPLEMENT or leave empty if no resources need to be freed */
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    /* TODO:IMPLEMENT */
	_delay_ms(1);
	TWI0.MADDR = ((address << 1) | 0x01);			
	while (!(TWI0.MSTATUS & TWI_RIF_bm));

	if ((TWI0.MSTATUS & TWI_RXACK_bm)) 
	{
		TWI0.MCTRLB = TWI_MCMD_STOP_gc;
		return -1;
	}
	
	int i;
	for(i = 0; i < (count - 1); i++)
	{
		while((TWI0.MSTATUS & TWI_RIF_bm) == 0);			
		data[i] = (TWI0.MDATA) ;					
		TWI0.MCTRLB = 0x02; 
	}
	
	while((TWI0.MSTATUS & 0x80) == 0);			
	data[i] = (TWI0.MDATA) ;					
	
	TWI0.MCTRLB = 0x07; 
		
    return 0;
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data,uint8_t count) 
{
    /* TODO:IMPLEMENT */ 
	while((TWI0.MSTATUS & 0x03) != 0x01);
	
	TWI0.MADDR = ((address << 1) | 0x00);	// send slave address and write command
	
	int i;
	for(i=0; i<count; i++)
	{
		while (!(TWI0.MSTATUS & TWI_WIF_bm));
		if ((TWI0.MSTATUS & TWI_RXACK_bm)) 
		{
			TWI0.MCTRLB = TWI_MCMD_STOP_gc;
			return -1;
		}
		TWI0.MDATA = data[i];
	}
	
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MCTRLB |= TWI_MCMD_STOP_gc; //send stop bit
		
    return 0;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) 
{
    /* TODO:IMPLEMENT */
    int i;
    for (i = 0; i < (useconds / 10); i++) 
	{
	    _delay_us(10);
    }
}
