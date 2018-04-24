/*
 * TWI_Master.c
 *
 * Created: 12/3/2015 2:06:47 PM
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
#include "TWI_Master.h"

// Choose an address for Master & Slave
#define SLAVE_ADDRESS	0x0F	// This can be any address other than 0x00
#define MASTER_ADDRESS	0x18	// This can be any address other than 0x00

// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x00
#define TWI_CMD_MASTER_READ  0x01

// TWI Data Buffer
static unsigned char TWI_buf[ TWI_BUFFER_SIZE ];    // Transceiver buffer
static unsigned char TWI_msgSize;                   // Number of bytes to be transmitted.
static unsigned char TWI_bufPtr;

// File stream for UART. Used for Transmission to demonstrate the fprintf function.
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// ADC variables
volatile unsigned int Ain;

// ----------------------------------------------------------------------------------
// Starts ADC conversion and waits
void start_ADC_and_wait()
{
	// Start A to D conversion
	ADCSRA |= (1<<ADSC);
	
	// Wait until this conversion is completed
	while((ADCSRA & (1<<ADSC)));
}

// ----------------------------------------------------------------------------------
// TWI Master Transmitter Mode State Machine (Blocking)
void TWI_Master_Transmiter_State_Machine()
{
	// Generate Start Condition.
	TWCR =	(1<<TWEN) |				// TWI Interface enabled.
			(1<<TWINT)|				// Clear the flag.
			(1<<TWSTA);				// Initiate a START condition.
	
	// This loop breaks once transmission is complete
	while(1)
	{
		switch (TWSR & 0xF8)
		{
			case TWI_START:             // START has been transmitted
			TWI_bufPtr = 0;				// Set buffer pointer to the TWI Address location
			
			case TWI_MTX_ADR_ACK:       // SLA+W has been transmitted and ACK received
			case TWI_MTX_DATA_ACK:      // Data byte has been transmitted and ACK received
			if (TWI_bufPtr < TWI_msgSize)
			{
				TWDR = TWI_buf[TWI_bufPtr++];
				TWCR =	(1<<TWEN)	|					// TWI Interface enabled
						(1<<TWINT);						// Clear the flag to send byte
			}
			else						// Send STOP after last byte
			{
				TWCR =	(1<<TWEN) |						// TWI Interface enabled
						(1<<TWINT)|						// Clear the flag
						(1<<TWSTO);						// Initiate a STOP condition.
				return;					// Transmission completed
			}
			break;
			
			case TWI_MTX_ADR_NACK:		// SLA+W has been transmitted and NACK received
			case TWI_MTX_DATA_NACK:		// Data byte has been transmitted and NACK received
			TWCR =	(1<<TWEN) |						// TWI Interface enabled
					(1<<TWINT)|						// Clear the flag
					(1<<TWSTO);						// Initiate a STOP condition.
			continue;
			
			default:					// In case of any problems, restart.
			TWCR =	(1<<TWEN) |						// TWI Interface enabled.
					(1<<TWINT)|						// Clear the flag.
					(1<<TWSTA);						// Initiate a START condition.
			break;
		}
		
		// Wait for TWINT Flag to set.
		while (!(TWCR & (1<<TWINT)));
	}
}

// ----------------------------------------------------------------------------------
// Initialize I2C Master
void TWI_Master_Initialize(void)
{
	TWAR = (MASTER_ADDRESS << 1);	// A unique address. Don't need it until data is to be received.
	TWBR = 32;						// With Prescaler @ 1, and F_CPU=16MHz, this generates F_SCL=200KHz
	TWDR = 0xFF;					// Default content = SDA released.
	TWCR = (1<<TWEN);				// Enable TWI
}

// ----------------------------------------------------------------------------------
// Transmit one byte to a slave over I2C
void TWI_Master_Transmit(uint8_t Address, uint8_t Data)
{
	TWI_msgSize = 2;                        // Number of bytes (address & data) to transmit.
	TWI_buf[0]  =	(Address<<1)|			// Store slave address with R/W setting.
					TWI_CMD_MASTER_WRITE;	
	TWI_buf[1]  = Data;						// Store Data to be transmitted
	
	TWI_Master_Transmiter_State_Machine();	// Initiate transmission.
}

// ----------------------------------------------------------------------------------
// All initializations
void initialize_all(void)
{
	// Initialize UART
	uart_init();                         // Initialize UART
	stdout = stdin = stderr = &uart_str; // Set File outputs to point to UART stream
	printf("Hello! \n");

	// ADC Setup
	ADMUX |= (1<<MUX2) | (1<<MUX1);						// Select ADC Channel 6
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);		// Set ADC prescaler to 128
	ADCSRA |= (1<<ADEN);								// Enable ADC circuit
	
	// Initialize I2C
	TWI_Master_Initialize();
}

// ----------------------------------------------------------------------------------
int main(void)
{
	// Initialize All
	initialize_all();
	
	while(1)
	{
		// Convert A to D
		start_ADC_and_wait();
		
		// Read the ADC value
		Ain = (ADCL);		// First read lower byte
		Ain |= (ADCH<<8);	// Then read upper byte
		
		// Send one byte to Slave
		TWI_Master_Transmit(SLAVE_ADDRESS, (Ain>>2));
		
		// Print sent data on UART
		printf("Sent Data: %u \n", Ain>>2 );
		_delay_ms(100);
	}
}