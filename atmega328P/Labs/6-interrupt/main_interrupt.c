/*
 * selftrial_lab6_interrupt.c
 *
 * Created: 3/17/2018 8:23:20 PM
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

void initialize_interrupt(void) {
	// Rising edge of the button generates interrupt
	EICRA |= (1 << ISC11) | (1 << ISC10);
	// Enable INT1
	EIMSK |= (1 << INT1);
	// Enable global interrupt
	sei();
	// External Interrupt Flag
	EIFR |= (1 << INTF1);
}

void initialize_all(void) {
	DDRD = 0xFF;			// Set PORTD as output
	PORTB |= (1 << PB1);	// Pull-up resistor for the switch in PB1
	DDRD  &= ~(1<<DDD3);	// INT1 Switch is input
	
	// Initialize uart
	uart_init();
	stdout = stdin = stderr = &uart_str;

	initialize_ADC();		// Initialize ADC
	initialize_interrupt();	// Initialize Interrupt
}


void blinkLED(void) {
	PORTD = 0b00000000;
	_delay_ms(250);
	PORTD = 0b00100000;
	_delay_ms(250);
}

void turn_LED_off(int off_time) {
	char state = PORTD;
	PORTD = 0b00000000;
	_delay_ms(off_time);
	PORTD = state;
}


// Interrupt Function for External Interrupt INT1
ISR(INT1_vect) {
	turn_LED_off(5000);
}

int main(void) {
	initialize_all();
	
	while (1) {
		blinkLED();	
	}
}

