/*
 * LabTest5.c
 *
 * Created: 4/23/2018 2:33:13 PM
 * Author : Tanvir-Laptop
 */ 


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

#define SW_PRESSED !(PIND & (1<<PIND3))

#define LM75_ADDRESS_W			0b10010000
#define LM75_ADDRESS_R			0b10010001
#define LM75_TEMP_REGISTER		0b00000000
#define LM75_CONFIG_REGISTER	0b00000001
#define LM75_THYST_REGISTER		0b00000010
#define LM75_TOS_REGISTER		0b00000011


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


// ADC variables
volatile unsigned int ain_temp;
volatile float voltage_temp;
float current_temp_ADC;

volatile unsigned int ain_photo;
volatile float voltage_photo;
int current_photo_ADC;

// I2C variables
uint8_t tempHighByte, tempLowByte;

// Setup mode variables
char adc_temp[2] = "T1";
char i2c_temp[2] = "T2";
char adc_photo[1] = "B";
char pwm_output[3];
char spi_output[3];
	
// DAC Variables
uint16_t DAC_data;

// For Real time of 0.001s, 16bit clock needs 8000 ticks
volatile uint16_t time_period = 15999;		// 8000-1
volatile uint16_t duty_cycle  = 0;			// on time
int percent_dutycycle;



void initialize_interrupt(void) {
	// Falling edge of the button generates interrupt
	EICRA |= (1 << ISC11);
	//EICRA |= (1 << ISC11) | (1 << ISC10);
	// Enable INT1
	EIMSK |= (1 << INT1);
	// External Interrupt Flag
	//EIFR |= (1 << INTF1);
}


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


void initialize_UART(void) {
	// Initialize UART
	uart_init();
	stdout = stdin = stderr = &uart_str; 
	printf("\n\nStarting Temp Reading... \n");
}


void initialize_I2C(void) {
	clock_prescale_set(clock_div_1); // 8MHz
	initI2C();
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


void initialize_timer(void) {
	// Set up Timer1
	OCR1A = time_period;	//Set Compare reg A
	OCR1B = duty_cycle;		//Compare reg B
	//OCR1A   = duty_cycle;
	
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


void initialize_all(void) {
	DDRD |= 0b10000000;
	DDRD |= 0b01000000;
	
	//PORTB |= (1 << PB1);	// Pull-up resistor for the switch in PB1
	DDRD  &= ~(1<<DDD3);	// INT1 Switch is input
	
	current_temp_ADC  = 0.00;
	current_photo_ADC = 0.00;
	
	initialize_LCD();
	initialize_UART();
	initialize_interrupt();
	sei();
	initialize_ADC();
	initialize_I2C();
	initialize_SPI();
	initialize_DAC();
	initialize_timer();
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
	
	//total_temp_ADC = total_temp_ADC + current_temp_ADC;
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
	
	//total_photo_ADC = total_photo_ADC + current_photo_ADC;
}

void read_i2c_temp(void) {
	// To set register, address LM75 in write mode
	i2cStart();
	i2cSend(LM75_ADDRESS_W);
	i2cSend(LM75_TEMP_REGISTER);
	i2cStart();
	
	//Setup and send address, with read bit
	i2cSend(LM75_ADDRESS_R);
	
	// Now receive two bytes of temperature
	tempHighByte = i2cReadAck();
	tempLowByte  = i2cReadNoAck();
	i2cStop();
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

void DAC_T1_for_LED(void) {
	// Normalize to DAC_DATA_MAX range
	float Vout_spl =  (float) ((current_temp_ADC - 15.00) / 20.00 ) * 5.00;
	DAC_data  = (Vout_spl * 1.00 * DAC_DATA_MAX) / 5;
	printf("Vout_spl = %f\n", Vout_spl);
	printf("DAC_data = %u\n", DAC_data);
	
	// Prepare DAC command along with data
	DAC_data |= (SELECT_DAC_A | VREF_BUFF | GAIN_1X | OUTPUT_ENABLED);
	
	// Send Command & Data to DAC over SPI
	SPI_Transmitter_for_DAC(DAC_data);
}

void DAC_T2_for_LED(void) {
	// Normalize to DAC_DATA_MAX range
	float Vout_spl =  (float) (((float)tempHighByte - 15.00) / 20.00 ) * 5.00;
	DAC_data  = (Vout_spl * 1.0 * DAC_DATA_MAX) / 5;
	//printf("Vout_spl = %f\n", Vout_spl);
	//printf("DAC_data = %u\n", DAC_data);
	
	// Prepare DAC command along with data
	DAC_data |= (SELECT_DAC_A | VREF_BUFF | GAIN_1X | OUTPUT_ENABLED);
	
	// Send Command & Data to DAC over SPI
	SPI_Transmitter_for_DAC(DAC_data);
}

void DAC_B_for_LED(void) {
	// Normalize to DAC_DATA_MAX range
	DAC_data  = (current_photo_ADC * 1.0 * DAC_DATA_MAX) / 10;
	//printf("Vout_spl = %f\n", current_photo_ADC);
	//printf("DAC_data = %u\n", DAC_data);
	
	// Prepare DAC command along with data
	DAC_data |= (SELECT_DAC_A | VREF_BUFF | GAIN_1X | OUTPUT_ENABLED);
	
	// Send Command & Data to DAC over SPI
	SPI_Transmitter_for_DAC(DAC_data);
}


void calculate_duty_cycle_photo(float reading) {
	if (reading < 1)
		percent_dutycycle = 5; 
	else if (reading > 9)
		percent_dutycycle = 95;
	else
		percent_dutycycle = floor(reading * 10);
		
	duty_cycle = ((1+time_period) * (percent_dutycycle * 0.01)) - 1;
}

void calculate_duty_cycle_temp(float reading) {
	if (reading < 15)
		percent_dutycycle = 5;
	else if (reading > 35)
		percent_dutycycle = 95;
	else
		percent_dutycycle = floor(((reading - 15.00) / 20.00) * 100.00);
	
	duty_cycle = ((1+time_period) * (percent_dutycycle * 0.01)) - 1;
}

void setup_mode(void) {
	printf("\n\n*** Setup Mode ***\n---------------------\n\n");
	printf("Input Options\nT1 : (ADC Based Temperature)\nT2 : (I2C Based Temperature)\nT3 : (ADC Based Photosensor)\n\n");
	printf("Output Options\nSPI (LED Connected)\nPWM (LED Connected)\n\n");
	
	//memset(pwm_output, 0, strlen(pwm_output));
	//memset(spi_output, 0, strlen(spi_output));
	
	printf("Which input do you want as the PWM output? ");
	scanf("%[^\n]%*c", pwm_output);
	printf("Which input do you want as the SPI output? ");
	scanf("%[^\n]%*c", spi_output);
	
	
	printf("PWM is %s, SPI is %s\n", pwm_output, spi_output);
	
	_delay_ms(1000);
}


void execution_mode(void) {
	// READ AND PRINT ADC TEMPARATURE
	start_ADC_and_wait();
	read_temp_sensor();
	printf("T1(C) = %0.2f  ", current_temp_ADC);
	
	
	// READ AND PRINT I2C TEMPARATURE
	read_i2c_temp();
	printf("T2(C) = %0.2f  ", (float)tempHighByte);
	
	
	// READ AND PRINT ADC PHOTO SENSOR
	start_ADC_and_wait();
	read_photo_sensor();
	printf("B = %d\n\n", current_photo_ADC);
	
	// Calculate DAC
	if (spi_output[0] == 'T' && spi_output[1] == '1') {
		DAC_T1_for_LED();
	}
	else if (spi_output[0] == 'T' && spi_output[1] == '2') {
		DAC_T2_for_LED();
	}
	else if (spi_output[0] == 'B') {
		//printf("DAC B\n");
		DAC_B_for_LED();
	}
	
	// Calculate PWM duty cycle
	if (pwm_output[0] == 'T' && spi_output[1] == '1') {
		calculate_duty_cycle_temp(current_temp_ADC);
	}
	else if (pwm_output[0] == 'T' && spi_output[1] == '2') {
		calculate_duty_cycle_temp((float)tempHighByte);
	}
	else if (pwm_output[0] == 'B') {
		calculate_duty_cycle_photo(current_photo_ADC);
	}
	
	
}

// Interrupt Function for External Interrupt INT1
ISR(INT1_vect) {
	EIMSK &= ~(1<<INT1);
	//while (SW_PRESSED);
	_delay_ms(100);
	setup_mode();
	_delay_ms(100);
	EIMSK |= (1<<INT1);
}


//Timer 1 compare match A ISR
ISR (TIMER1_COMPA_vect){
	PORTD = 0b10000000;		//set Port D
	OCR1B = duty_cycle;		//load new duty cycle
}

//Timer 1 compare match B ISR
ISR(TIMER1_COMPB_vect){
	PORTD = 0x00;		//Port D cleared
}


int main(void) {
	initialize_all();
	setup_mode();
	
	while (1) {
		execution_mode();
		while (SW_PRESSED);
	}
		
	return (0);
}