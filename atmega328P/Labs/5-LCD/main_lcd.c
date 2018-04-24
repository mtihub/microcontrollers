#define F_CPU 16000000UL	/* Tells the Clock Freq to the Compiler. */
#include <avr/io.h>			/* Defines pins, ports etc. */
#include <util/delay.h>		/* Functions to waste time */
#include "lcd_lib.h"		/* LCD Library */

int main(void) {
	// Initialize LCD
	initialize_LCD();
	
	LcdDataWrite('L');
	LcdDataWrite('C');
	LcdDataWrite('D');
	LcdDataWrite(' ');
	LcdDataWrite('I');
	LcdDataWrite('n');
	LcdDataWrite('i');
	LcdDataWrite('t');
	LcdDataWrite('i');
	LcdDataWrite('a');
	LcdDataWrite('l');
	LcdDataWrite('i');
	LcdDataWrite('z');
	LcdDataWrite('e');
	LcdDataWrite('d');
	LcdDataWrite(' ');
	LCDGotoXY(0, 1);
	
	// Setting Port D as output
	DDRD = 0xff;
	
	// Initialize counter variables
	int counter      = 0;
	int isIncreasing = 1;
	
	while (1) {
		if ( !(PINB & (1 << PINB1)) ) {
			if (counter == 0) 
				isIncreasing = 1;
			if (counter == 16)		// Max characters in one row
				isIncreasing = 0;
			
			if (isIncreasing == 1) {
				// Print asterisk
				LcdDataWrite('*');
				counter++;
			}
			else {
				// Remove an asterisk
				LCDcursorLeft(1);
				LcdDataWrite(' ');
				LCDcursorLeft(1);
				counter--;
			}
			
			// Wait (Debounce)
			_delay_ms(500);		
		}
	}
	
	return (0);
}




/*
int main(void) {
	// Initialize LCD
	initialize_LCD();
	
	// Setting Port D as output
	DDRD = 0xff;
	
	// Initialize counter variables

	while (1) {
		if ( !(PINB & (1 << PINB1)) ) {
			int printChar = 65;
			LcdDataWrite(printChar);
			_delay_ms(400);
			
			while ( !(PINB & (1 << PINB1)) ) {
				LCDcursorLeft(1);
				printChar++;
				LcdDataWrite(printChar);
				_delay_ms(400);
			}
			
			if ( !(PINB & (1 << PINB1)) ) {
				
			}
			
			// Wait (Debounce)
			_delay_ms(700);
		}
	}
	
	return (0);
}
*/