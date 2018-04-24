/*
 * LabTest3.c
 *
 * Created: 4/9/2018 2:22:23 PM
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


// UART File
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


volatile unsigned int Ain;
volatile float Voltage;

float total_temp_in_cel_ADC = 0.00;
float total_temp_in_far_ADC = 0.00;

float current_reading_in_cel_ADC;

float avg_temp_in_cel_ADC;
float avg_temp_in_far_ADC;

char lcd_tempcel_buffer[17];
char lcd_tempfar_buffer[17];

unsigned long total_time = 0;
unsigned long laps = 0;
int pause = 0;
unsigned long pause_time = 0;


// For Real time of 0.001s, 16bit clock needs 8000 ticks
volatile uint16_t time_period = 7999;		// 8000-1
volatile uint16_t duty_cycle = 0;			// on time
int percent_dutycycle;

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
	// External Interrupt Flag
	EIFR |= (1 << INTF1);
}

void initialize_timer(void) {
	// Set up Timer0 in CTC mode to overflow after 1ms at 16MHz/64 Clock frequency
	OCR0A = 249; 					// Set the compare reg to 250 time ticks
	TIMSK0 |= (1<<OCIE0A);			// turn on Timer0 Compare match ISR
	TCCR0A |= (1<<WGM01);			// turn on clear-on-match, CTC mode
	TCCR0B = (1<<CS01) | (1<<CS00); // Set pre-scalar to divide by 64
	
	
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



// Function to initialize everything
void initialize_all() {
	DDRD = 0xFF;			// Set PORTD as output
	PORTB |= (1 << PB1);	// Pull-up resistor for the switch in PB1
	DDRD  &= ~(1<<DDD3);	// INT1 Switch is input
	
	// Initialize uart
	uart_init();
	stdout = stdin = stderr = &uart_str;
	printf("\n\nStarting Temp Reading... \n");
	
	// Initialize LCD
	initialize_LCD();

	initialize_ADC();		// Initialize ADC
	initialize_interrupt();	// Initialize Interrupt
	// Enable global interrupt
	sei();
	initialize_timer();		// Initialize Timer
}


void read_temp(void) {
	start_ADC_and_wait();
	// READ ADC
	Ain = ADCL;
	Ain |= (ADCH << 8);
	
	Voltage = Ain / 1024.00 * 5.00;
	current_reading_in_cel_ADC = (float)(Voltage - 0.400) / 0.0195; 
	
	total_temp_in_cel_ADC = total_temp_in_cel_ADC + current_reading_in_cel_ADC;
	
	total_temp_in_far_ADC = total_temp_in_far_ADC + ((float)(current_reading_in_cel_ADC * 1.80) + 32.00);
}

void get_avg_temp(void) {
	avg_temp_in_cel_ADC = total_temp_in_cel_ADC / 5.00;
	avg_temp_in_far_ADC = total_temp_in_far_ADC / 5.00;
}


void display_temp_and_update_duty_cycle(void) {
	// Clear LCD
	LCDclr();
	
	// Setup frequency string and display it
	char cel_buffer[64];
	char far_buffer[64];
	//char percent_dutycycle_buffer[2];
	
	snprintf(cel_buffer, sizeof cel_buffer, "%f", avg_temp_in_cel_ADC);
	snprintf(far_buffer, sizeof far_buffer, "%f", avg_temp_in_far_ADC);
	
	LcdDataWrite('T');
	LcdDataWrite('(');
	LcdDataWrite('C');
	LcdDataWrite(')');
	LcdDataWrite('=');
	
	int i, dot_found = 0, char_from_dot = 0;
	for (i = 0; i < strlen(cel_buffer); i++) {
		if (cel_buffer[i] == '.') {
			dot_found++;
			//dot_found_at = i;
		}
		
		LcdDataWrite(cel_buffer[i]);
		
		if (dot_found > 0) {
			char_from_dot++;
		}
		if (char_from_dot >= 3) {
			break;
		}
	}
	
	LCDGotoXY(0,1);
	LcdDataWrite('T');
	LcdDataWrite('(');
	LcdDataWrite('F');
	LcdDataWrite(')');
	LcdDataWrite('=');
	
	dot_found = 0;
	char_from_dot = 0;
	for (i = 0; i < strlen(far_buffer); i++) {
		if (cel_buffer[i] == '.') {
			dot_found++;
		}
		
		LcdDataWrite(far_buffer[i]);
		
		if (dot_found > 0) {
			char_from_dot++;
		}
		if (char_from_dot >= 3) {
			break;
		}
	}

	
	float temp = (avg_temp_in_cel_ADC - (float) floor(avg_temp_in_cel_ADC)) * 100.00;
	percent_dutycycle = (int) floor(temp);
	duty_cycle = ((1+time_period) * (percent_dutycycle * 0.01)) - 1;
	//printf("\n\nPERCENT DUTY CYCLE: %d\nDUTY CYCLE: %d\n", percent_dutycycle, duty_cycle);
	
	/*
	percent_dutycycle_buffer[0] = cel_buffer[dot_found_at+1];
	percent_dutycycle_buffer[1] = cel_buffer[dot_found_at+2];
	percent_dutycycle = atoi(percent_dutycycle_buffer);
	duty_cycle = ((percent_dutycycle * (1 + time_period)) / 100) - 1;
	printf("\n\nPERCENT DUTY CYCLE: %d\n", percent_dutycycle);

	
	sprintf(lcd_tempcel_buffer, "T(C)=%0.2f", avg_temp_in_cel_ADC);
	sprintf(lcd_tempfar_buffer, "T(F)=%0.2f", avg_temp_in_far_ADC);
	
	LCDstring((uint8_t*) lcd_tempcel_buffer, strlen(lcd_tempcel_buffer));
	LCDGotoXY(0,1);
	LCDstring((uint8_t*) lcd_tempfar_buffer, strlen(lcd_tempfar_buffer));
	*/
}

/*
void update_duty_cycle(void) {
	percent_dutycycle = (avg_temp_in_cel_ADC - (float)floor(avg_temp_in_cel_ADC)) * 100;
	
	
}
*/


// Timer compare interrupt
ISR (TIMER0_COMPA_vect) {
	if (pause == 1) {
		pause_time++;
		if (pause_time == 3000) {
			pause = 0;
			pause_time = 0;
			total_time = 0;
			PORTD = 0b00000000;
		}
	}
	else {
		total_time++;
	
		if (total_time >= 40) {
			read_temp();
			laps++;
		
		
			if (laps == 5) {
				get_avg_temp();
				//update_duty_cycle();
				display_temp_and_update_duty_cycle();
				//printf("ADC Temp: %f\n%f\n", avg_temp_in_cel_ADC, avg_temp_in_far_ADC);
				total_temp_in_cel_ADC = 0;
				total_temp_in_far_ADC = 0;
			
				laps = 0;
			}
		
			total_time = 0;
		}
	}
}

// Interrupt Function for External Interrupt INT1
ISR(INT1_vect) {
	pause = 1;
	PORTD = 0b00100000;
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


int main(void)
{
	initialize_all();

	//read_temp();
	//display_temp();
	//printf("ADC Temp: %0.2f\n%0.2f\n", tempcel_ADC, tempfar_ADC);
	
    while (1)  {
		
	}
}

