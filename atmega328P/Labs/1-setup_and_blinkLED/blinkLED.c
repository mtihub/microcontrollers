/*
 * blinkLED.c
 *
 * Created: 1/23/2018 1:27:23 AM
 * Author : Tanvir-Laptop
 */ 

#define F_CPU 16000000UL	/* Tells the Clock Freq to the Compiler. */
#include <avr/io.h>			/* Defines pins, ports etc. */
#include <util/delay.h>		/* Functions to waste time */

int main(void) {
	/* Configure LED pin as output */
	DDRB |= 1<<DDB5;
	
	/* Data Direction Register D: Setting Port D as output. */
	DDRD = 0b11111111;
	
	/* ------ Event loop ------ */
	while (1) {
		PORTB |= 1<<PORTB5;		/* switch on inbuilt the LED*/
		PORTD = 0b01010101;		/* Turn on alternate LEDs in PORTD */
		_delay_ms(1000);		/* wait for 1 second */
		
		PORTB &= ~(1<<PORTB5);	/* switch off the inbuilt LED */
		PORTD = 0b10101010;		/* Toggle the LEDs */
		_delay_ms(1000);		/* wait for 1 second */
	} 
	/* End event loop */
	
	return (0); /* This line is never reached */
}