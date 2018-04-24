/*
 * selftrial_lab5_ADC.c
 *
 * Created: 3/17/2018 7:30:57 PM
 * Author : Tanvir-Laptop
 */ 


#define F_CPU 16000000
#define __DELAY_BACKWARD_COMPATIBLE__

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart.h"

#define VREF 5.00

volatile unsigned int a_in;
volatile float voltage;

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


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

void initialize_all(void) {
	DDRD = 0xFF;			// Set PORTD as output
	PORTB |= (1 << PB1);	// Pull-up resistor for the switch in PB1
	initialize_ADC();		// Initialize ADC
	
	// Initialize uart
	uart_init();
	stdout = stdin = stderr = &uart_str;
}

void read_ADC(void) {
	a_in = ADCL;			// ADCL holds the lower 8 bits
	a_in |= (ADCH << 8);	// ADCH holds the higher 8 bits

	voltage = ((float)a_in / 1024.00) * VREF;		// Formula: ADC = (Vin * 1024) / Vref
}

void blinkLED(int on, int off) {
	PORTD = 0b00000100;
	_delay_ms(on);
	PORTD = 0b00000000;
	_delay_ms(off);
}

int main(void) {
	initialize_all();

	// Variables
	double freq   = 50;	// Given Frequency: 50 Hz
	float  period = pow(freq, -1.00) * 1000;		// Period in ms
	float  duty_cycle;
	int    on_time, off_time;

	while (1) {
		start_ADC_and_wait();
		read_ADC();
		
		duty_cycle = (float) (voltage / VREF);
		on_time    = (int) (duty_cycle * period);
		off_time   = (int) ((1-duty_cycle) * period); 
	
		//printf("a_in: %d\nperiod = %d\non time = %d\noff time = %d\n\n", a_in, (int)period, on_time, off_time);
		
		blinkLED(on_time, off_time);
	}
}
