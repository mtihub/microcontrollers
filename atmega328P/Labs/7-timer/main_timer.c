/*
 * selftrial_lab8_timers.c
 *
 * Created: 3/17/2018 10:15:47 PM
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
#include "uart.h"
#include "lcd_lib.h"

#define SW_PRESSED !(PIND & (1 << PIND3))

#define RESET_STATE   0
#define ON_STATE      1
#define DISPLAY_STATE 2

char  LCD_time_buffer[17];
const uint8_t LCD_Ready[]    PROGMEM = "Ready!";
const uint8_t LCD_Counting[] PROGMEM = "Counting...";
const uint8_t LCD_Time[]     PROGMEM = "Total Time: ";

unsigned long total_time = 0;
int current_state = RESET_STATE;


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
	// Falling edge of the button generates interrupt
	EICRA |= (1 << ISC11);
	// Enable INT1
	EIMSK |= (1 << INT1);
	// Enable global interrupt
	sei();
	// External Interrupt Flag
	EIFR |= (1 << INTF1);
}

void initialize_timer(void) {
	// CTC MODE
	TCCR0A |= (1 << WGM01);
	// Clear Output Compare pin (OC0A) on Compare Match
	TCCR0A |= (1 << COM0A1);
	// Pre-scalar of 64
	TCCR0B |= (1 << CS01) | (1 << CS00);
	// Reset timer when tick count reaches (250-1)
	OCR0A = 249;
	// Output Compare Match A Interrupt Enable
	TIMSK0 |= (1 << OCIE0A);
}

void initialize_all(void) {
	DDRD = 0xFF;			// Set PORTD as output
	PORTB |= (1 << PB1);	// Pull-up resistor for the switch in PB1
	DDRD  &= ~(1<<DDD3);	// INT1 Switch is input
	
	// Initialize uart
	uart_init();
	stdout = stdin = stderr = &uart_str;
	
	// Initialize LCD
	initialize_LCD();

	initialize_ADC();		// Initialize ADC
	initialize_interrupt();	// Initialize Interrupt
	initialize_timer();		// Initialize Timer
}

// Interrupt Service Routine for Timer
ISR(TIMER0_COMPA_vect) {
	total_time++;
}

// Interrupt Service Routine for External Interrupt INT1
ISR(INT1_vect) {
	// Disable INT1
	EIMSK &= ~(1 << INT1);
	
	if (current_state == RESET_STATE) {
		total_time = 0;
	}
}

void display_time(void) {
	CopyStringtoLCD(LCD_Time, 0, 0);
	sprintf(LCD_time_buffer, "%lu ms", total_time);
	LCDGotoXY(0,1);
	LCDstring((uint8_t*) LCD_time_buffer, (strlen(LCD_time_buffer)));
}

void check_state(void)
{
	if (SW_PRESSED)
	{
		while (SW_PRESSED);
		
		switch (current_state)
		{
			case RESET_STATE:
			CopyStringtoLCD(LCD_Counting, 0, 0);
			current_state = ON_STATE;
			break;
			case ON_STATE:
			display_time();
			current_state = DISPLAY_STATE;
			break;
			case DISPLAY_STATE:
			LCDclr();
			CopyStringtoLCD(LCD_Ready, 0, 0);
			current_state = RESET_STATE;
			break;
		}
		
		_delay_ms(100);
	}
	
	// Re-enable the interrupt
	EIMSK |= (1 << INT1);
}

int main(void) {
	initialize_all();
	
	while (1) {
		check_state();
	}
}

