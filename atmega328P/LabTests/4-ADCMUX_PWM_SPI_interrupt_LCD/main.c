/*
 * LabTest4.c
 *
 * Created: 4/16/2018 2:33:34 PM
 * Author : Tanvir-Laptop
 */ 

#define F_CPU 16000000UL
#define __DELAY_BACKWARD_COMPATIBLE__

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"
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

// LCD 
#define LCD_DDRAM           	7	//DB7: set DD RAM address
#define LCD_LINE0_DDRAMADDR		0x00
#define LCD_LINE1_DDRAMADDR		0x40
#define DATA_PORT				PORTC
#define DATA_PORT_DDR			DDRC
#define CTRL_PORT				PORTC
#define CTRL_PORT_DDR			DDRC
#define RS						4
#define ENABLE					5

// File stream for UART. Used for Transmission to demonstrate the fprintf function.
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// LCD variables
char lcd_temp_buffer[17];
char lcd_photo_buffer[17];

// ADC variables
volatile unsigned int ain_temp;
volatile float voltage_temp;
float current_temp_ADC;
float total_temp_ADC;
float avg_temp_ADC;

volatile unsigned int ain_photo;
volatile float voltage_photo;
int current_photo_ADC;
int total_photo_ADC;
int avg_photo_ADC;

// Timer0 Variables
unsigned long total_time = 0;
unsigned long total_time_no_reset = 0;
unsigned long laps = 0;
int pause = 0;
unsigned long pause_time = 0;

// DAC Variables
uint16_t DAC_data;

// PWM Variables
// Freq = 5kHz, For Real time of 0.0002s, 16bit clock needs 3200 ticks
volatile uint16_t time_period = 3199;		// 3200-1
volatile uint16_t duty_cycle = 0;			// on time
int percent_dutycycle = 0;


//-------------------------------------------------------------------------------------------------------------
// LCD FUNCTIONS
//-------------------------------------------------------------------------------------------------------------
void LcdDataWrite_NoBlocking(uint8_t da) {
	int i;

	// First send higher 4-bits
	DATA_PORT = (DATA_PORT & 0xf0) | (da >> 4); //give the higher half of cm to DATA_PORT
	CTRL_PORT |= (1<<RS);		//setting RS=1 to choose the data register
	CTRL_PORT |= (1<<ENABLE);	//setting ENABLE=1
	i = 0;
	while (i++ < 1000);
	CTRL_PORT &= ~(1<<ENABLE); // Setting ENABLE=0
	i = 0;
	while (i++ < 1000);
	// Send lower 4-bits
	DATA_PORT = (DATA_PORT & 0xf0) | (da & 0x0f); //give the lower half of cm to DATA_PORT
	CTRL_PORT |= (1<<RS);		//setting RS=1 to choose the data register
	CTRL_PORT |= (1<<ENABLE);	//setting ENABLE=1
	i = 0;
	while (i++ < 1000);
	CTRL_PORT &= ~(1<<ENABLE); // Setting ENABLE=0
	i = 0;
	while (i++ < 1000);
}

void LCDstring_NoBlocking(uint8_t* data, uint8_t nBytes)	//Outputs string to LCD
{
	register uint8_t i;

	// check to make sure we have a good pointer
	if (!data) return;

	// print data
	for(i=0; i<nBytes; i++)
	{
		LcdDataWrite_NoBlocking(data[i]);
	}
}


void LcdCommandWrite_NoBlocking(uint8_t cm)
{
	//unsigned long capture_time;
	int i;
	// First send higher 4-bits
	DATA_PORT = (DATA_PORT & 0xf0) | (cm >> 4); //give the higher half of cm to DATA_PORT
	CTRL_PORT &= ~(1<<RS);		//setting RS=0 to choose the instruction register
	CTRL_PORT |= (1<<ENABLE);	//setting ENABLE=1
	i = 0;
	while (i++ < 1000);
	CTRL_PORT &= ~(1<<ENABLE); // Setting ENABLE=0
	i = 0;
	while (i++ < 1000);
	// Send lower 4-bits
	DATA_PORT = (DATA_PORT & 0xf0) | (cm & 0x0f); //give the lower half of cm to DATA_PORT
	CTRL_PORT &= ~(1<<RS);		//setting RS=0 to choose the instruction register
	CTRL_PORT |= (1<<ENABLE);	//setting ENABLE=1
	i = 0;
	while (i++ < 1000);
	CTRL_PORT &= ~(1<<ENABLE); // Setting ENABLE=0
	i = 0;
	while (i++ < 1000);
}


void LCDclr_NoBlocking(void) //Clears LCD
{
	LcdCommandWrite_NoBlocking(0x01);
}


void LCDGotoXY_NoBlocking(uint8_t x, uint8_t y) //Cursor to X Y position
{
	register uint8_t DDRAMAddr;
	// remap lines into proper order
	switch(y)
	{
		case 0: DDRAMAddr = LCD_LINE0_DDRAMADDR+x; break;
		case 1: DDRAMAddr = LCD_LINE1_DDRAMADDR+x; break;
		default: DDRAMAddr = LCD_LINE0_DDRAMADDR+x;
	}
	// set data address
	LcdCommandWrite_NoBlocking(1<<LCD_DDRAM | DDRAMAddr);
}

//-------------------------------------------------------------------------------------------------------------




void initialize_ADC(void) {
	// Set ADMUX
	ADMUX &= ~((1 << MUX2) | (1 << MUX1) | (1 << MUX0));
	
	// Set Pre-scalar of 128
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// Start ADC by writing 1 to ADEN in ADCSRA
	ADCSRA |= (1 << ADEN);
}

void set_ADC6(void) {
	// ADC6: MUX3..0 = 0110
	ADMUX &= ~(1 << MUX0);
	ADMUX |= (1 << MUX2) | (1 << MUX1);
}

void set_ADC7(void) {
	// ADC6: MUX3..0 = 0110
	ADMUX |= (1 << MUX2) | (1 << MUX1) | (1 << MUX0);
}

void start_ADC_and_wait(void) {
	// Start ADC Conversion by setting ADSC bit to 1
	ADCSRA |= (1 << ADSC);
	// Wait until the conversion is finished.
	// ADSC bit is 1 as long as the conversion is in progress
	while (ADCSRA & (1 << ADSC));
}


void initialize_timer(void) {
	// Set PORTD as output
	DDRD |= 0b00000100;
	
	// Set up Timer0 in CTC mode to overflow after 1ms at 16MHz/64 Clock frequency
	OCR0A = 249; 					// Set the compare reg to 250 time ticks
	TIMSK0 |= (1<<OCIE0A);			// turn on Timer0 Compare match ISR
	TCCR0A |= (1<<WGM01);			// turn on clear-on-match, CTC mode
	TCCR0B = (1<<CS01) | (1<<CS00); // Set pre-scalar to divide by 64
	
	// Set up Timer1
	OCR1A = time_period;	//Set Compare reg A
	OCR1B = duty_cycle;		//Compare reg B

	// Fast PWM with TOP = OCR1A, BOTTOM = BOTTOM, OVF = TOP
	TCCR1A |= (1<<WGM11) | (1<<WGM10);		//fast PWM A
	TCCR1B |= (1<<WGM13) | (1<<WGM12);		//fast PWM B
	
	// Set OC1A/OC1B on Compare Match
	// clear OC1A/OC1B at BOTTOM (inverting mode)
	TCCR1A |= (1 << COM1A1);
	TCCR1A |= (1 << COM1B1);
	
	// No Pre-scalar
	TCCR1B |= (1 << CS10);
	
	// Output Compare Interrupt enabled
	TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);
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
	// Global variables
	total_temp_ADC  = 0.00;
	total_photo_ADC = 0;
	
	// Initialize UART
	uart_init();                        
	stdout = stdin = stderr = &uart_str; 
	printf("\n\nStarting Photo Sensor Reading... \n");
	
	// Initialize LCD
	initialize_LCD();
	LCDcursorOFF();
	
	// Run other initialization functions
	initialize_ADC();
	sei();
	initialize_timer();
	initialize_SPI();
	initialize_DAC();
}



// READ ADC VALUES
void read_temp_sensor(void) {
	// READ TEMP
	set_ADC6();
	start_ADC_and_wait();
	
	ain_temp = ADCL;
	ain_temp |= (ADCH << 8);
	
	voltage_temp = ain_temp / 1024.00 * 5.00;
	current_temp_ADC = (float) ((voltage_temp - 0.400) / 0.0195);
	//printf("ADC Temp: %f\n", current_temp_ADC);
	
	total_temp_ADC = total_temp_ADC + current_temp_ADC;
}

void read_photo_sensor(void) {
	// READ PHOTO
	set_ADC7();
	start_ADC_and_wait();
		
	ain_photo = ADCL;
	ain_photo |= (ADCH << 8);
		
	voltage_photo = ain_photo / 1024.00 * 5.00;
	current_photo_ADC = floor(voltage_photo / 0.5);
	//printf("ADC Photo: %d\n\n", current_photo_ADC);
	
	total_photo_ADC = total_photo_ADC + current_photo_ADC;
}

void get_average_readings(void) {
	avg_temp_ADC  = total_temp_ADC  / 6.00;
	avg_photo_ADC = total_photo_ADC / 6.00;
}


// Sends a 16bit word (command + data) to DAC over SPI
void SPI_Transmitter_for_DAC(uint16_t digital_data) {
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

void DAC_for_LED(void) {
	// Normalize to DAC_DATA_MAX range
	DAC_data  = (avg_photo_ADC * 1.0 * DAC_DATA_MAX) / 10;
	
	// Prepare DAC command along with data
	DAC_data |= (SELECT_DAC_A | VREF_BUFF | GAIN_1X | OUTPUT_ENABLED);
	
	// Send Command & Data to DAC over SPI
	SPI_Transmitter_for_DAC(DAC_data);
}

void calculate_duty_cycle(void) {
	/*
	if (avg_photo_ADC < 1) {
		percent_dutycycle = 5;
	}
	if (avg_photo_ADC > 8) {
		percent_dutycycle = 95;
	}
	else {
		percent_dutycycle = avg_photo_ADC * 10;
	}
	printf("Percent Dutycycle: %d\n\n", percent_dutycycle);
	*/
	percent_dutycycle = avg_photo_ADC * 10;
	if (percent_dutycycle == 0) 
		duty_cycle = 0;
	else if (percent_dutycycle == 100) 
		duty_cycle = time_period;
	else 
		duty_cycle = ((1+time_period) * (percent_dutycycle * 0.01)) - 1;
}


// LCD FUNCTIONS
// USE NO-BLOCKING FOR CONTINUOUS PWM
void display_readings(void) {
	LCDclr();
	//LCDclr_NoBlocking();
	
	sprintf(lcd_temp_buffer, " Temp=%0.2f", avg_temp_ADC);
	sprintf(lcd_photo_buffer, "Brightness=%d", avg_photo_ADC);
	
	LCDstring((uint8_t*) lcd_temp_buffer, strlen(lcd_temp_buffer));
	LCDGotoXY(0,1);
	LCDstring((uint8_t*) lcd_photo_buffer, strlen(lcd_photo_buffer));
	
	//LCDstring_NoBlocking((uint8_t*) lcd_temp_buffer, strlen(lcd_temp_buffer));
	//LCDGotoXY_NoBlocking(0,1);
	//LCDstring_NoBlocking((uint8_t*) lcd_photo_buffer, strlen(lcd_photo_buffer));
}



// Interrupt Service Routine for Timer0
ISR (TIMER0_COMPA_vect) {
	total_time++;
	total_time_no_reset++;
		
	if (total_time >= 50) {
		read_temp_sensor();
		read_photo_sensor();
		laps++;	
			
		if (laps == 6) {
			get_average_readings();
			DAC_for_LED();
			//calculate_duty_cycle();
			display_readings();
			//printf("ADC Avg Temp: %f\nADC Avg Photo: %d\n\n", avg_temp_ADC, avg_photo_ADC);
			
			// Reset variables
			total_temp_ADC  = 0.00;
			total_photo_ADC = 0.00;
			laps = 0;
		}
			
		// Reset total time
		total_time = 0;
	}
}


// Interrupt Service Routine for Timer1
//Timer 1 compare match A ISR
ISR (TIMER1_COMPA_vect){
	PORTD = 0b00000100;		//set Port D
	calculate_duty_cycle();
	OCR1B = duty_cycle;		//load new duty cycle
}

//Timer 1 compare match B ISR
ISR(TIMER1_COMPB_vect){
	PORTD = 0x00;		//Port D cleared
}


int main(void) {
	initialize_all();
	
	while (1) {
	
	}

}