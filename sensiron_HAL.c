#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 4000000UL
#include <util/delay.h>

typedef enum 
{
	TWI_STATE_IDLE,
	TWI_STATE_WRITE,
	TWI_STATE_READ
} twi_state_t;

volatile twi_state_t twi_state = TWI_STATE_IDLE;
volatile uint8_t twi_buffer[32];
volatile uint8_t twi_length = 0;
volatile uint8_t twi_index = 0;
volatile uint8_t twi_error = 0;
volatile uint8_t twi_address = 0;
volatile bool twi_transfer_complete = false;

void sensirion_i2c_hal_init(void) 
{
	TWI0.MBAUD = 0x01; 
	TWI0.MCTRLA = TWI_ENABLE_bm ;
	TWI0.DBGCTRL = 0x01;
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

void sensirion_i2c_hal_free(void) //dont need to implement
{

}

ISR(TWI0_TWIM_vect) 
{
	cli();
	
	if ((twi_state == TWI_STATE_WRITE) && (TWI0.MSTATUS & TWI_WIF_bm)) 
	{
		
		if (twi_index < twi_length) {
			TWI0.MDATA = twi_buffer[twi_index++];
			} else {
			TWI0.MCTRLB = TWI_MCMD_STOP_gc;//done writing all bits, send stop bit 
			twi_transfer_complete = true; //transfer completed
			TWI0.MCTRLA &= ~TWI_WIEN_bm; //disable interrupt 
		}	
	}
	else if ((twi_state == TWI_STATE_READ) && (TWI0.MSTATUS & TWI_RIF_bm)) {
		if (twi_index < twi_length - 1) {
			twi_buffer[twi_index++] = TWI0.MDATA;
			TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc; // ACK and continue
			
		} 
		else {
			twi_buffer[twi_index++] = TWI0.MDATA;
			TWI0.MCTRLB = 0x07; // NACK and STOP
			twi_transfer_complete = true;
			TWI0.MCTRLA &= ~TWI_RIEN_bm; //disable receive interrupt 
		}
		
	}
	
	sei();
}

static void start_write(uint8_t address, const uint8_t* data, uint8_t count) {
	while((TWI0.MSTATUS & 0x03) != 0x01); // wait until idle 
	twi_state = TWI_STATE_WRITE;
	TWI0.MCTRLA |= TWI_WIEN_bm;
	
	sei();
	
	twi_address = address;
	twi_index = 0;
	twi_length = count;
	twi_transfer_complete = false;
	twi_error = 0;

	for (uint8_t i = 0; i < count; i++) 
	{
		twi_buffer[i] = data[i];
	}

	TWI0.MADDR = (twi_address << 1)
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint8_t count) 
{
	
	start_write(address, data, count);

	while (!twi_transfer_complete){}; // Wait for ISR to complete

	if (twi_error) 
	{
		return -1;
	}

	return 0;
}

static void start_read(uint8_t address, uint8_t count) 
{
	twi_state = TWI_STATE_READ;
	TWI0.MCTRLA |= TWI_RIEN_bm;
	
	sei();
	twi_address = address;
	twi_index = 0;
	twi_length = count;
	twi_transfer_complete = false;
	twi_error = 0;

	TWI0.MADDR = (twi_address << 1) | 1; // Read mode
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
	
	start_read(address, count);

	while (!twi_transfer_complete); // Wait for ISR to complete

	if (twi_error) {
		return -1;
	}

	for (uint8_t i = 0; i < count; i++) {
		data[i] = twi_buffer[i];
	}

	return 0;
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
	while (useconds >= 10) {
		_delay_us(10);
		useconds -= 10;
	}
 }
