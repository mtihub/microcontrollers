/*
 * lAB11Temp.c
 *
 * Created: 3/5/2018 3:11:05 PM
 * Author : Tanvir-Laptop
 */ 

// ------- Preamble -------- //
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/power.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd_lib.h"
#include "pinDefines.h"
#include "uart.h"
#include "i2c.h"
#include "TWI_Slave.h"

// -------- Defines -------- //
#define LM75_ADDRESS_W			0b10010000
#define LM75_ADDRESS_R			0b10010001
#define LM75_TEMP_REGISTER		0b00000000
#define LM75_CONFIG_REGISTER	0b00000001
#define LM75_THYST_REGISTER		0b00000010
#define LM75_TOS_REGISTER		0b00000011

// File stream for UART. Used for Transmission to demonstrate the fprintf function.
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// PWM variables
volatile unsigned int Ain;
volatile float Voltage;


void start_ADC_and_wait() {
	ADCSRA |= (1 << ADSC);
	while ((ADCSRA & (1 << ADSC)));
}

int main(void) {
	initialize_LCD();
	
	uint8_t tempHighByte, tempLowByte;
	uint8_t temp_ADC;
	
	// -------- Inits --------- //
	clock_prescale_set(clock_div_1); /* 8MHz */
	
	// Initialize UART
	uart_init();                         // Initialize UART
	stdout = stdin = stderr = &uart_str; // Set File outputs to point to UART stream
	printf("\n\nStarting Temp Reading... \n");

	// ADC
	ADMUX |= (1 << MUX2) | (1 << MUX1);
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);

	initI2C();

	// ------ Event loop ------ //
	while (1) {
		// START ADC
		start_ADC_and_wait();
		
		// READ ADC
		Ain = ADCL;
		Ain |= (ADCH << 8);
		
		Voltage = Ain / 1024.00 * 5.00;
		temp_ADC = (Voltage - 0.400) / 0.0195;
		printf("ADC Temp: %u\n", temp_ADC);
		
		/* To set register, address LM75 in write mode */
		i2cStart();
		i2cSend(LM75_ADDRESS_W);
		i2cSend(LM75_TEMP_REGISTER);
		i2cStart();						/* restart, just send start again */
		
		/* Setup and send address, with read bit */
		i2cSend(LM75_ADDRESS_R);
		
		/* Now receive two bytes of temperature */
		tempHighByte = i2cReadAck();
		tempLowByte = i2cReadNoAck();
		i2cStop();
		
		
		// Print it out nicely over serial for now...
		//printByte(tempHighByte);
		printf("I2C Temp: %d \n\n", tempHighByte);
		
		/* Once per second */
		_delay_ms(1000);
	} /* End event loop */
		
	return (0); /* This line is never reached */
}


/*
// ----------------------------------------------------------------------------------

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
*/