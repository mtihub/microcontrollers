/*
 * TWI_Slave.c
 *
 * Created: 12/3/2015 2:07:24 PM
 *  Author: Syed Kamran Haider
 *			With the help of Atmel Corporation's AppNote
 *			AVR315 - TWI Master Implementation
 */


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"
#include "TWI_Slave.h"

// Choose an address for Master & Slave
#define SLAVE_ADDRESS	0x0F	// This can be any address other than 0x00
#define MASTER_ADDRESS	0x18	// This can be any address other than 0x00

// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x00
#define TWI_CMD_MASTER_READ  0x01

// TWI Data Buffer
static unsigned char TWI_buf[ TWI_BUFFER_SIZE ];    // Transceiver buffer
static unsigned char TWI_bufPtr;

// File stream for UART. Used for Transmission to demonstrate the fprintf function.
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// ----------------------------------------------------------------------------------
// Function to print TWI Status Codes for debugging
void ERROR(uint8_t Error_code)
{
	// For now, this function prints only the error code to UART
	// This function can be extended to implement different
	// error handling based on the received error code
	printf("I2C Error Code: 0x%x \n", Error_code);
}

// ----------------------------------------------------------------------------------
// TWI Master Transmitter Mode State Machine (Blocking)
void TWI_Slave_Receiver_State_Machine()
{
	// Enable TWI.
	TWCR =	(1<<TWEN) |		// TWI Interface enabled.
			(1<<TWINT)|		// Clear the flag.
			(1<<TWEA);		// Acknowledge on any new requests.
	
	// This loop breaks once a reception is complete
	while(1)
	{
		// Wait for TWINT Flag to set.
		while (!(TWCR & (1<<TWINT)));
		
		//ERROR(TWSR & 0xF8);		// For Debugging
		switch (TWSR & 0xF8)
		{
			case TWI_SRX_GEN_ACK:			// General call address has been received; ACK has been returned
			case TWI_SRX_ADR_ACK:			// Own SLA+W has been received, ACK has been returned
			TWI_bufPtr   = 0;				// Set buffer pointer to first data location
			// Re-Enable TWI.
			TWCR =	(1<<TWEN) |		// TWI Interface enabled.
					(1<<TWINT)|		// Clear the flag.
					(1<<TWEA);		// Acknowledge on any new requests.
			break;
			
			case TWI_SRX_ADR_DATA_ACK:       // Previously addressed with own SLA+W; data has been received; ACK has been returned
			case TWI_SRX_GEN_DATA_ACK:       // Previously addressed with general call; data has been received; ACK has been returned
			TWI_buf[TWI_bufPtr++]	= TWDR;		// Read the data
			// Re-Enable TWI.
			TWCR =	(1<<TWEN) |		// TWI Interface enabled.
					(1<<TWINT)|		// Clear the flag.
					(1<<TWEA);		// Acknowledge on any new requests.
			break;
			
			case TWI_SRX_STOP_RESTART:       // A STOP condition or repeated START condition has been received while still addressed as Slave
			// Re-Enable TWI.
			TWCR =	(1<<TWEN) |		// TWI Interface enabled.
					(1<<TWINT)|		// Clear the flag.
					(1<<TWEA);		// Acknowledge on any new requests.
			return;				// Reception of message complete
			
			default:					// In case of any problems, restart.
			// Re-Enable TWI.
			TWCR =	(1<<TWEN) |		// TWI Interface enabled.
					(1<<TWINT)|		// Clear the flag.
					(1<<TWEA);		// Acknowledge on any new requests.
			break;
		}
	}
}

// ----------------------------------------------------------------------------------
// Initialize I2C Slave
void TWI_Slave_Initialize(uint8_t Address)
{
	TWAR = (Address << 1);		// Load Slave Address into TWAR Register.
	TWCR = (1<<TWEN);			// Enable TWI.
}

// ----------------------------------------------------------------------------------
// Transmit one byte to a slave over I2C
uint8_t TWI_Slave_Poll_Receive()
{	
	TWI_Slave_Receiver_State_Machine();	// Wait for data to arrive.
	return TWI_buf[0];					// Return Data
}

// ----------------------------------------------------------------------------------
// All initializations
void initialize_all(void)
{
	// Initialize UART
	uart_init();                         // Initialize UART
	stdout = stdin = stderr = &uart_str; // Set File outputs to point to UART stream
	printf("Hello! \n");
	
	// Initialize I2C
	TWI_Slave_Initialize(SLAVE_ADDRESS);
}

// ----------------------------------------------------------------------------------
int main(void)
{
	// Initialize All
	initialize_all();
	
	while(1)
	{
		// Poll for data over I2C. (Blocking)
		uint8_t data = TWI_Slave_Poll_Receive();
		
		// Print received data on UART
		printf("Received Data: %u \n", data );
	}
}