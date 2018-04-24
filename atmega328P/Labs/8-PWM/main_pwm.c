/*
 * selftrial_lab9_PWM.c
 *
 * Created: 3/18/2018 5:07:56 PM
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
#include "lcd_lib.h"

#define SW1_PRESSED !(PINB & (1 << PINB7))
#define SW2_PRESSED !(PINB & (1 << PINB1))

#define DUTY_CYCLE_INCREMENT 159

//LCD stuff
char lcd_buffer[17];	//LCD display buffer
const uint8_t LCD_Freq[] PROGMEM = "Freq: ";
const uint8_t LCD_Duty[] PROGMEM = "DutyCyc: ";

// Frequency = 1000 Hz
// For Real time of 0.001s, 16bit clock needs 16000 ticks
volatile uint16_t time_period = 15999;	// 16000-1
volatile uint16_t duty_cycle  = 7999;	// on time = half the period
int percent_dutycycle;

/*
void init_ADC(void)
{
	ADMUX  |= (1 << MUX2) | (1 << MUX1);
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);
}

void start_ADC_and_wait(void)
{
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
}
*/

void init_interrupt(void)
{
	DDRD  &= ~(1 << DDD3);
	EICRA |= (1 << ISC11);
	EIMSK |= (1 << INT1);
	sei();
}

void init_timer(void)
{
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
	
	// No Prescalar
	TCCR1B |= (1 << CS10);
	
	// Output Compare Interrupt enabled
	TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);
	
	//TIMSK1 |= (1 << TOIE1);
}

void initialize_all(void)
{
	DDRD = 0xff;		//LEDs output
	DDRB |= (1<<DDB2);	//PB2, the individual LED
	
	//time_period = 15999;
	//duty_cycle  = 7999;
	
	//init_ADC();
	init_interrupt();
	init_timer();
	initialize_LCD();
}

/*
ISR(TIMER1_OVF_vect)
{
	
}
*/




//Timer 1 compare match A ISR
ISR (TIMER1_COMPA_vect){
	PORTD = 0xFF;		//set Port D
	OCR1B = duty_cycle;	//load new duty cycle
}

//Timer 1 compare match B ISR
ISR(TIMER1_COMPB_vect){
	PORTD = 0x00;		//Port D cleared
}


//Print percentage duty cycle
void print_duty_cycle(void){
	//Percent duty cycle
	percent_dutycycle = ((1+duty_cycle)*100.0)/(1+time_period);
	
	//convert into a string and store in LCD buffer
	sprintf(lcd_buffer, "%i  ", percent_dutycycle);
	
	//Print
	LCDGotoXY(10,1);
	LCDstring(lcd_buffer, strlen(lcd_buffer));
}

//How duty cycle is updated
void update_duty_cycle(void){
	if(SW1_PRESSED){		//Duty cycle incrementing
		if(duty_cycle <= time_period - DUTY_CYCLE_INCREMENT)
			duty_cycle += DUTY_CYCLE_INCREMENT;
		print_duty_cycle();
	}
	else if(SW2_PRESSED){	//Duty cycle decrementing
		if(duty_cycle >= DUTY_CYCLE_INCREMENT)
			duty_cycle -= DUTY_CYCLE_INCREMENT;
		print_duty_cycle();
	}
}


int main(void)
{
	initialize_all();	//initialize
	
	//Print PWM freq on LCD
	CopyStringtoLCD(LCD_Freq, 0, 0);
	sprintf(lcd_buffer, "%i kHz", 1);
	LCDGotoXY(10, 0);
	LCDstring(lcd_buffer, strlen(lcd_buffer));
	
	//Print starting duty cycle
	CopyStringtoLCD(LCD_Duty, 0, 1);
	print_duty_cycle();
	
	while (1)
	{
		update_duty_cycle();
		_delay_ms(100);
	}
}
