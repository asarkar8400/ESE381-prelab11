//***************************************************************************
//
// File Name : "scd41_basic.c"
// Title : Design Task 2
// Date : 05/01/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// Sensirion HAL functions using interrupts to display co2ppm, temp, and rh
// Warnings :
// Restrictions : none
// Algorithms : none
// References :
//
// Revision History : Initial version
//
//
//**************************************************************************

#include "sensirion_i2c_hal.h"     
#include "sensirion_common.h"     
#include "sensirion_config.h"      
#include <stdbool.h>               
#include <avr/io.h>                
#include <avr/interrupt.h>        
#define F_CPU 4000000UL            
#include <util/delay.h>           

//TWI FSM states
typedef enum
{
	TWI_STATE_IDLE,    // IDLE state
	TWI_STATE_WRITE,   // Writing State
	TWI_STATE_READ     // Reading StAte
} twi_state_t;

volatile uint8_t twi_buffer[32];                 // I2C buffer
volatile uint8_t twi_address = 0;                // Slave address
volatile uint8_t twi_error = 0;                  // Error flag
volatile uint8_t twi_length = 0;                 // Number of bytes to transfer
volatile uint8_t twi_index = 0;                  // Current byte index
volatile twi_state_t twi_state = TWI_STATE_IDLE; // Current state
volatile bool twi_transfer_complete = false;     // Completion flag

//***************************************************************************
// Function Name : "sensirion_i2c_hal_init"
// Date : 05/01/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function configures the use of I2C on the AVR128DB48 with the SCD41
// Revision History : Initial version
//**************************************************************************
void sensirion_i2c_hal_init(void)
{
	TWI0.MBAUD = 0x01;						// Set baud rate  
	TWI0.MCTRLA = TWI_ENABLE_bm;			// Enable TWI master
	TWI0.DBGCTRL = 0x01;					// Enable debug
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;	// Force bus state to idle
}

//No need to implement this 
void sensirion_i2c_hal_free(void)
{
}

// ISR to handle writes and reads
ISR(TWI0_TWIM_vect)
{
	cli(); // Disable interrupts during ISR 

	// Handles writes
	if ((twi_state == TWI_STATE_WRITE) && (TWI0.MSTATUS & TWI_WIF_bm))
	{
		// If there are still bytes to send
		if (twi_index < twi_length) {
			TWI0.MDATA = twi_buffer[twi_index++]; // Send next byte
			} else {
			TWI0.MCTRLB = TWI_MCMD_STOP_gc;       // Send STOP condition
			twi_transfer_complete = true;         // Signal transfer complete
			TWI0.MCTRLA &= ~TWI_WIEN_bm;          // Disable write interrupt
		}
	}
	// Hhandles reads
	else if ((twi_state == TWI_STATE_READ) && (TWI0.MSTATUS & TWI_RIF_bm))
	{
		// Read next byte with ACK if not last
		if (twi_index < twi_length - 1) {
			twi_buffer[twi_index++] = TWI0.MDATA;     // Read received byte
			TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;      // Send ACK, continue
		}
		else {
			twi_buffer[twi_index++] = TWI0.MDATA;     // Read final byte
			TWI0.MCTRLB = 0x07;                       // Send NACK + STOP
			twi_transfer_complete = true;             // Signal complete
			TWI0.MCTRLA &= ~TWI_RIEN_bm;              // Disable read interrupt
		}
	}

	sei(); // renable interuppts
}

//***************************************************************************
// Function Name : "TWI_start_write"
// Date : 05/01/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function begins write operations
// Revision History : Initial version
//**************************************************************************
static void TWI_start_write(uint8_t address, const uint8_t* data, uint8_t count)
{
	while((TWI0.MSTATUS & 0x03) != 0x01); // Wait for bus to be idle

	twi_state = TWI_STATE_WRITE;          // Set write state
	TWI0.MCTRLA |= TWI_WIEN_bm;           // Enable write interrupt
	sei();                                // Enable global interrupts

	twi_address = address;                // Set slave address
	twi_index = 0;                        // Start at byte 0
	twi_length = count;                   // Set byte count
	twi_transfer_complete = false;        // Clear transfer flag
	twi_error = 0;                        // Clear errors

	// Copy data to internal buffer
	for (uint8_t i = 0; i < count; i++)
	{
		twi_buffer[i] = data[i];
	}

	TWI0.MADDR = (twi_address << 1);      // Send address andwrite bit
}

//***************************************************************************
// Function Name : "sensirion_i2c_hal_write"
// Date : 05/01/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function writes data
// Revision History : Initial version
//**************************************************************************
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint8_t count)
{
	TWI_start_write(address, data, count);        // Begin writing

	while (!twi_transfer_complete) {};			//wait for writing transfer to finish

	if (twi_error) {
		return -1;                            // Return error 
	}

	return 0;                                 
}

//***************************************************************************
// Function Name : "TWI_start_read"
// Date : 05/01/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function begins read operations
// Revision History : Initial version
//**************************************************************************
static void TWI_start_read(uint8_t address, uint8_t count)
{
	twi_state = TWI_STATE_READ;              // Set read state
	TWI0.MCTRLA |= TWI_RIEN_bm;              // Enable read interrupt
	sei();                                   // Enable global interrupts

	twi_address = address;                   // Set slave address
	twi_index = 0;                           // Start at byte 0
	twi_length = count;                      // Set number of bytes
	twi_transfer_complete = false;           // Clear done flag
	twi_error = 0;                           // Clear error

	TWI0.MADDR = (twi_address << 1) | 1;     // Send address + read bit (R/W=1)
}

//***************************************************************************
// Function Name : "sensirion_i2c_hal_read"
// Date : 05/01/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// This function  reads data
// Revision History : Initial version
//**************************************************************************
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count)
{
	TWI_start_read(address, count);              // Begin read

	while (!twi_transfer_complete);          // Wait for ISR to signal done

	if (twi_error) {
		return -1;                           // Error occurred
	}

	// Copy data to user buffer
	for (uint8_t i = 0; i < count; i++) {
		data[i] = twi_buffer[i];
	}

	return 0;                                // Success
}

//***************************************************************************
// Function Name : "sensirion_i2c_hal_sleep_usec"
// Date : 05/01/2025
// Version : 1.0
// Target MCU : AVR128DB48
// Target Hardware ;
// Author : Aritro Sarkar
// DESCRIPTION
// Essentially a dealy function
// Revision History : Initial version
//**************************************************************************
void sensirion_i2c_hal_sleep_usec(uint32_t useconds)
{
	while (useconds >= 10) {
		_delay_us(10);                       // Delay 10 µs per loop
		useconds -= 10;
	}
}
