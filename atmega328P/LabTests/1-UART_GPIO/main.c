/*
 * LabTest1.c
 *
 * Created: 3/26/2018 2:34:01 PM
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


#define SW1_PRESSED !(PINB & (1 << PINB7))		// Switch 1: Internal Switch
#define SW2_PRESSED !(PINB & (1 << PINB1))		// Switch 2: External Switch connected to PB1

// UART File
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// Switch 1 (internal switch) and 2 (external switch) 

// Global variables
int delay_period_ms, freq, pos, total_time, perform_check;

// Function to initailize everything
void initialize_all() {
	// Set PORTD as output
	DDRD = 0b11111111;
	
	// Initialize counter variable
	total_time = 0;
	perform_check = 0;
		
	// Init uart
	uart_init();
	stdout = stdin = stderr = &uart_str;
	
	// Set up Timer0 in CTC mode to overflow after 1ms at 16MHz/64 Clock frequency
	OCR0A = 249; 					// Set the compare reg to 250 time ticks
	TIMSK0 |= (1<<OCIE0A);			// turn on Timer0 Compare match ISR
	TCCR0A |= (1<<WGM01);			// turn on clear-on-match, CTC mode
	TCCR0B = (1<<CS01) | (1<<CS00); // Set pre-scalar to divide by 64
	
	// Enable interrupt
	sei();
	
	// LCD
	initialize_LCD();
}


// Timer compare interrput
ISR (TIMER0_COMPA_vect) {
	// Increment Total Time ticks
	total_time++;
	
	// Check UART every 15 seconds
	if (total_time == 15000) {
		perform_check = 1;
	}
}


// Take input in the UART
void take_input(void) {
	printf("\nEnter Frequency: ");
	scanf("%d", &freq);
	printf("Enter Position: ");
	scanf("%d", &pos);
	getchar();
}


// Task a -> blink LED
void task_a(void) {
	// Period (in ms) = 1/Freq * 1000;
	delay_period_ms = pow((double)freq, -1.00) * 1000;
	
	// Blink LED
	PORTD = 0b00000000;
	_delay_ms(delay_period_ms);
	switch (pos) {
		case 1:
			PORTD = 0b10000000;
			break;
		case 2:
			PORTD = 0b01000000;
			break;
		case 3:
			PORTD = 0b00100000;
			break;
		case 4:
			PORTD = 0b00010000;
			break;
		case 5:
			PORTD = 0b00001000;
			break;
		case 6:
			PORTD = 0b00000100;
			break;
	}
	_delay_ms(delay_period_ms);
}


// Task b and c -> press switch to change LED / Frequency
void task_b_c(void) {
	if (SW1_PRESSED && SW2_PRESSED) {
		if (pos == 1) pos = 6;
		else pos--;
		_delay_ms(100);
	}
	else if (SW1_PRESSED) {
		if (freq < 15) freq++;
		_delay_ms(100);
	}
	else if (SW2_PRESSED) {
		if (freq > 1) freq--;
		_delay_ms(100);
	}
}

// Check UART if the user wants to change LED and Frequency
void check_uart(void) {
	if (perform_check == 1) {
		perform_check = 0;
		
		char choice;
		printf("Change setup? (y/n): ");
		scanf("%c", &choice);
		getchar();
		
		if (choice == 'y')
			take_input();
		
		total_time = 0;
	}
}


int main(void)
{
	// Initialize all
	initialize_all();
	
	// Take initial input
    take_input();
	
	while (1) 
    {
		// Task a: Blink the LEDs
		task_a();
		
		// Task b and c: Check switch press to change LED / Frequency
		task_b_c();
		
		// Task c: Check UART input to check if the user wants to change the LED/Frequency
		check_uart();
    }
}

