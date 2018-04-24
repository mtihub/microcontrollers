/*
 * LabTest2.c
 *
 * Created: 4/2/2018 2:31:16 PM
 * Author : Tanvir-Laptop
 */ 


#define F_CPU 16000000
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

#define START_STATE				0
#define SETUP_STATE				1
#define FREQUENCY_TOGGLE_STATE	2
#define POSITION_TOGGLE_STATE	3
#define EXECUTION_STATE			4

#define STARTING_FREQUENCY 3.00
#define DUTY_CYCLE 0.50

const uint8_t LCD_setupmode[] PROGMEM = "Set up Mode";
const uint8_t LCD_sw1press[]  PROGMEM = "Press SW 1";
const uint8_t LCD_frequency[] PROGMEM = "Frequency";
const uint8_t LCD_position[]  PROGMEM = "Position";

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

int		freq, position;
float	period_ms; 
int		on_time, off_time;
int		current_state;

// Take input in the UART
void take_input(void) {
	printf("\nEnter Frequency: ");
	scanf("%d", &freq);
	printf("Enter Position: ");
	scanf("%d", &position);
	getchar();
}

void initialize_all() {
	DDRD = 0b11111111;		// Set PORTD as output
	PORTB |= (1 << PB1);	// Pull-up resistor for the switch in PB1
	
	// Initialize uart
	uart_init();
	stdout = stdin = stderr = &uart_str;
	
	// Initialize LCD
	initialize_LCD();

	// Take user input for frequency and LED position
	take_input();
	
	// Global Variables
	current_state = START_STATE;
}

// Blink LED at the specified position with the specified on and off time
void blink_LED(void) {
	// Calculate on time and off time
	period_ms = (float) pow((double)freq, -1.00) * 1000.00;
	on_time   = (int)(period_ms * DUTY_CYCLE);
	off_time  = (int)(period_ms * (1-DUTY_CYCLE));
	
	// Blink LED
	PORTD = 0b00000000;
	_delay_ms(off_time);
	switch (position) {
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
	_delay_ms(on_time);
}


// Display starting frequency and position in the LCD
void display_current_status(void) {
	// Clear LCD
	LCDclr();
	
	// Setup strings to display
	char lcd_frequency_buffer[17];
	char lcd_position_buffer[17];
	sprintf(lcd_frequency_buffer, "Frequency: %d Hz", freq);
	sprintf(lcd_position_buffer,  "Position : %d", position);
	
	// Display the strings
	LCDstring((uint8_t*) lcd_frequency_buffer, strlen(lcd_frequency_buffer));
	LCDGotoXY(0,1);
	LCDstring((uint8_t*) lcd_position_buffer, strlen(lcd_position_buffer));
}

// Display setup mode
void display_setup_state(void) {
	//clear LCD
	LCDclr();
	
	// Copy strings to display
	CopyStringtoLCD(LCD_setupmode, 0, 0);
	CopyStringtoLCD(LCD_sw1press,  0, 1);
}

// Display frequency toggle state
void display_frequency_toggle_state(void) {
	// Clear LCD
	LCDclr();
	
	// Display "Frequency"
	CopyStringtoLCD(LCD_frequency, 0, 0);
	LCDGotoXY(0,1);
	
	// Setup frequency string and display it
	char lcd_frequency_str[17];
	sprintf(lcd_frequency_str, "%d Hz", freq);
	LCDstring((uint8_t*) lcd_frequency_str, strlen(lcd_frequency_str));
}

// Display position toggle state
void display_position_toggle_state(void) {
	// Clear LCD
	LCDclr();
	
	// Display "Frequency"
	CopyStringtoLCD(LCD_position, 0, 0);
	LCDGotoXY(0,1);
	
	// Setup frequency string and display it
	char lcd_position_str[17];
	sprintf(lcd_position_str, "%d", position);
	LCDstring((uint8_t*) lcd_position_str, strlen(lcd_position_str));
}


// Display execution state
void display_execution_state(void) {
	// Clear LCD
	LCDclr();
	
	// Setup frequency and position string and display it
	char lcd_status_str[17];
	sprintf(lcd_status_str, "Freq:%dHz/Pos:%d", freq, position);
	LCDstring((uint8_t*) lcd_status_str, strlen(lcd_status_str));
	LCDGotoXY(0,1);
	
	// Create asterisks in the second row based on the current position
	char lcd_asterisk[17];
	switch (position) {
		case 1:
			sprintf(lcd_asterisk, "*");
			break;
		case 2:
			sprintf(lcd_asterisk, "* *");
			break;
		case 3:
			sprintf(lcd_asterisk, "* * *");
			break;
		case 4:
			sprintf(lcd_asterisk, "* * * *");
			break;
		case 5:
			sprintf(lcd_asterisk, "* * * * *");
			break;
		case 6:
			sprintf(lcd_asterisk, "* * * * * *");
			break;
	}
	LCDstring((uint8_t*) lcd_asterisk, strlen(lcd_asterisk));
}

// Switch press inputs
void check_switch_press(void) {
	
	if ((SW1_PRESSED && SW2_PRESSED) && (current_state == START_STATE || current_state == EXECUTION_STATE))  {
		while(SW1_PRESSED && SW2_PRESSED);
		
		current_state = SETUP_STATE;
		display_setup_state();
		_delay_ms(100);
	}
	
	if ((SW1_PRESSED && SW2_PRESSED) && (current_state != START_STATE)) {
		while(SW1_PRESSED && SW2_PRESSED);
		
		current_state = EXECUTION_STATE;
		display_execution_state();
		_delay_ms(100);
	}
	
	if ((SW1_PRESSED) && (current_state == SETUP_STATE)) {
		while(SW1_PRESSED);
		
		current_state = FREQUENCY_TOGGLE_STATE;
		display_frequency_toggle_state();
		_delay_ms(100);
	}
	
	if ((SW1_PRESSED) && (current_state == FREQUENCY_TOGGLE_STATE || current_state == POSITION_TOGGLE_STATE)) {
		while(SW1_PRESSED);
		
		if (current_state == FREQUENCY_TOGGLE_STATE) {
			current_state = POSITION_TOGGLE_STATE;
			display_position_toggle_state();
		}
		else if (current_state == POSITION_TOGGLE_STATE) {
			current_state = FREQUENCY_TOGGLE_STATE;
			display_frequency_toggle_state();
		}
			
		_delay_ms(100);
	}
	
	
	if ((SW2_PRESSED) && (current_state == FREQUENCY_TOGGLE_STATE)) {
		while(SW2_PRESSED);
		
		if (freq < 15) freq++;
		else freq = 1;
		
		display_frequency_toggle_state();
		_delay_ms(100);
	}
	
	if ((SW2_PRESSED) && (current_state == POSITION_TOGGLE_STATE)) {
		while(SW2_PRESSED);
		
		if (position < 6) position++;
		else position = 1;
		
		display_position_toggle_state();
		_delay_ms(100);
	}
}


int main(void)
{
	initialize_all();
	display_current_status();
	
	while (1) 
    {
		blink_LED();
		check_switch_press();
    }
}

