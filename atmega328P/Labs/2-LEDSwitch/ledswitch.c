/*
 * Lab3LEDTasks.c
 *
 * Created: 1/24/2018 3:55:28 PM
 * Author : Tanvir-Laptop
 */ 


#define F_CPU 16000000UL	/* Tells the Clock Freq to the Compiler. */
#define __DELAY_BACKWARD_COMPATIBLE__

#include <avr/io.h>			/* Defines pins, ports etc. */
#include <util/delay.h>		/* Functions to waste time */


int setDelayVal(int freq) {
	if (freq > 10) {
		return 1;
	}
	else {
		switch(freq) {
			case 1:
			return 1000;
			case 2:
			return 500;
			case 3:
			return 333;
			case 4:
			return 250;
			case 5:
			return 200;
			case 6:
			return 166;
			case 7:
			return 142;
			case 8:
			return 125;
			case 9:
			return 111;
			case 10:
			return 100;
		}
	}
}

// Part B
int main(void) {
	// Initialize pullup resistor on our input pin
	PORTB |= (1 << PB1);
	
	// Switch 1 as input
	//DDRB &= ~(1 << DDB7);
	// Switch 2 as input (PINB7)
	
	// Setting Port D as output
	DDRD = 0xff;
	
	int currentFreq = 2;
	int delayVal = 333;
	unsigned char onLED  = 0b00000100;
	int goRight = 1;
	
	while (1) {
		// PRESS INTERNAL BUTTON COMMAND: !(PINB & (1 << PINB7))
		PORTD = onLED;
		_delay_ms(delayVal);
		PORTD = 0b00000000;
		_delay_ms(delayVal);
		
		if ( !(PINB & (1 << PINB7)) ) {
			currentFreq -= 1;
			delayVal = setDelayVal(currentFreq);
		}
		
		if ( !(PINB & (1 << PINB1)) ) {
			currentFreq += 1;
			delayVal = setDelayVal(currentFreq);
		}
		
		
		if (!(PINB & (1 << PINB7)) && !(PINB & (1 << PINB1))) {
			if (onLED == 0b10000000) {
				goRight = 0;
				onLED = 0b01000000;
			}
			else if (onLED == 0b00000001) {
				goRight = 1;
				onLED = 0b00000010;
			}
			else {
				if (goRight == 1) {
					onLED += onLED;
				}
				else {
					onLED = onLED >> 0b00000001;
				}
			}
		}
		
	}
	
	return (0);
}

