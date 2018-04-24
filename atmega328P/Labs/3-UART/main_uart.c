/*
 * Lab3USART.c
 *
 * Created: 1/29/2018 4:18:44 PM
 * Author : Tanvir-Laptop
 */ 


#define TWO_HZ_HALF_CYCLE_us 250000
#define EIGHT_HZ_HALF_CYCLE_us 62500

#define F_CPU 16000000UL	/* Tells the Clock Freq to the Compiler. */
#define __DELAY_BACKWARD_COMPATIBLE__

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>		/* Functions to waste time */
#include "uart.h"


FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


void blinkLED(int freq) {
	if (freq == 2) {
		PORTD = 0b00000100;
		_delay_ms(250);
		PORTD = 0b00000000;
		_delay_ms(250);
	}
	else {
		PORTD = 0b00000100;
		_delay_ms(62);
		PORTD = 0b00000000;
		_delay_ms(62);
	}
}

int switchFreq(int currentFreq) {
	switch (currentFreq) {
		case 2:
			return 8;
		case 8:
			return 2;
	}
}

long switchCycleTime(long current_half_cycle_time) {
	switch (current_half_cycle_time) {
		case  TWO_HZ_HALF_CYCLE_us:
			return EIGHT_HZ_HALF_CYCLE_us;
		case EIGHT_HZ_HALF_CYCLE_us:
			return TWO_HZ_HALF_CYCLE_us;
	}
}

int main(void)
{
	// Initialize pullup resistor on our input pin
	PORTB |= (1 << PB1);
	
	// Make PORTD output
	DDRD = 0xff;
	
	// Initial Frequency
	int freq = 2;

	// Clock counter. For some reasone, clock() of <time.h> doesn't work
	long half_cycle_time = TWO_HZ_HALF_CYCLE_us;	// Initial cycle time
	//long threshold = 10000000;					// Ten seconds. But the command window has a loading time. So lowering the threshold
	long threshold = 5000000;						// Lowered Threashold
	long time_counter;
	
	// Character to store the decision
	char decision;
	
	// Init UART
	uart_init();
	stdout = stdin = stderr = &uart_str;
	printf("\n\nStarting frequency: 2 Hz\n\n");
	
	while (1) 
    {
		// Blink LED in the specified frequency
		blinkLED(freq);
		
		// Increment time
		time_counter += half_cycle_time;
		
		
		if (time_counter >= threshold) {
			// Take decision input
			printf("\nDo you want to change LED mode? (Y/N):  ");
			scanf("%c", &decision);
			getchar();	// Without this anomolies get printed
				
			if (decision == 'Y') {
				freq = switchFreq(freq);
				half_cycle_time = switchCycleTime(half_cycle_time);
			}
			
			time_counter = 0;	// Reset counter
		}
    }
}

