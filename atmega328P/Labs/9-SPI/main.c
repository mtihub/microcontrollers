/*
 * selftrial_Lab10_SPI.c
 *
 * Created: 4/14/2018 12:42:55 AM
 * Author : Tanvir-Laptop
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd_lib.h"

// MCP4921 DAC Related Definitions
// Command Bits
#define SELECT_DAC_A	0x0000 // MCP4921 only has DAC_A
#define SELECT_DAC_B	0x8000 // MCP4922 can choose between DAC_A and DAC_B
#define VREF_UNBUFF		0x0000 // Input VREF should be buffered
#define VREF_BUFF		0x4000 // Input VREF should be buffered
#define GAIN_2X			0x0000 // DAC Output gain 2x
#define GAIN_1X			0x2000 // DAC Output gain 1x
#define OUTPUT_DISABLED 0x0000 // DAC output disabled
#define OUTPUT_ENABLED	0x1000 // DAC output enabled

// SPI related definitions
#define SPI_DDR		DDRB
#define SPI_PORT	PORTB
#define SPI_SS		PORTB2
#define SPI_MOSI	PORTB3
#define SPI_MISO	PORTB4
#define SPI_SCK		PORTB5

// Utilities
#define DAC_DATA_MAX	4095	// Maximum value for DAC Data bits
#define LDAC_PIN		PORTB0	// DAC Latch Enable pin
#define LDAC_PIN_DDR	DDRB
#define LDAC_PIN_PORT	PORTB

// Globals Variables
// ADC variables
volatile unsigned int Ain;
uint16_t DAC_data;

// LCD Strings
char lcd_buffer[17]; // LCD display buffer
const uint8_t LCD_Master[] PROGMEM	= "Master: ";
const uint8_t LCD_Slave[] PROGMEM	= "Slave : ";


void initialize_ADC(void) {
	// Select ADC6: MUX3..0 = 0110
	ADMUX |= (1 << MUX2) | (1 << MUX1);
	// Set Pre-scalar of 128
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// Start ADC by writing 1 to ADEN in ADCSRA
	ADCSRA |= (1 << ADEN);
}

void start_ADC_and_wait(void) {
	// Start ADC Conversion by setting ADSC bit to 1
	ADCSRA |= (1 << ADSC);
	// Wait until the conversion is finished.
	// ADSC bit is 1 as long as the conversion is in progress
	while (ADCSRA & (1 << ADSC));
}

void initialize_SPI(void) {
	// Set SS, MOSI and SCK output
	SPI_DDR |= (1<<SPI_SS) | (1<<SPI_MOSI) | (1<<SPI_SCK);
	// Enable SPI, Master, fck/128
	SPCR	|= (1<<SPE) | (1<<MSTR)  |(1<<SPR1) | (1<<SPR0);
}

void initialize_DAC(void) {
	// Set LDAC pin as output
	LDAC_PIN_DDR  |= (1<<LDAC_PIN);
	// Initially LDAC pin is high
	LDAC_PIN_PORT |= (1<<LDAC_PIN);
}


void initialize_all(void) {
	// start the LCD
	initialize_LCD();
	LCDcursorOFF();
	LCDclr();
	CopyStringtoLCD(LCD_Master, 0, 0);
	CopyStringtoLCD(LCD_Slave, 0, 1);
	
	initialize_ADC();
	initialize_SPI();
	initialize_DAC();
}

// Sends a 16bit word (command + data) to DAC over SPI
void SPI_Transmitter_for_DAC(uint16_t digital_data)
{
	// Pull LDAC pin High
	LDAC_PIN_PORT |= (1<<LDAC_PIN);
	
	// Pull Slave_Select low
	SPI_PORT &= ~(1<<SPI_SS);
	
	// Send upper byte of data to DAC
	SPDR = (digital_data >> 8);
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	
	// Send Lower byte of data to DAC
	SPDR = (digital_data & 0xFF);
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	
	// Pull Slave Select High
	SPI_PORT |= (1<<SPI_SS);
	
	// Pull LDAC pin Low
	LDAC_PIN_PORT &= ~(1<<LDAC_PIN);
}

int main(void)
{
	initialize_all();
	
	while(1) {
		// Convert A to D
		start_ADC_and_wait();
		
		// Read the ADC value
		Ain	 = (ADCL);		// First read lower byte
		Ain |= (ADCH<<8);	// Then read upper byte
		
		// Normalize to DAC_DATA_MAX range
		DAC_data = (Ain * 1.0 * DAC_DATA_MAX) / 1024.0;
		
		//DAC_data = 3000;
		
		// Prepare DAC command along with data
		DAC_data |= (SELECT_DAC_A | VREF_BUFF | GAIN_1X | OUTPUT_ENABLED);
		
		// Send Command & Data to DAC over SPI
		SPI_Transmitter_for_DAC(DAC_data);
		
		// Print on LCD
		// ADC Reading
		sprintf(lcd_buffer, "%u ", Ain);
		LCDGotoXY(8,0);
		LCDstring((uint8_t*)lcd_buffer, strlen(lcd_buffer));
		
		// DAC Reading
		sprintf(lcd_buffer, "%u ", (DAC_data & 0x0FFF) );
		LCDGotoXY(8,1);
		LCDstring((uint8_t*)lcd_buffer, strlen(lcd_buffer));
		_delay_ms(100);
	}
}
