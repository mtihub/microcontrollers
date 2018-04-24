/*
 * RedBotOwnImpl.c
 *
 * Created: 3/20/2018 10:45:25 AM
 * Author : Tanvir-Laptop
 */ 

#define F_CPU 16000000UL	// CPU 16 MHz

// Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart.h"

// Voltage returned by the IR Sensors. 
// The threshold for the returned voltage to be considered as black
#define BLACK_THRESHOLD 4

// Optimized Speed for the motors

// Optimized for curvy S
// Increased speed. Straight path is wiggly but the turns are fast
#define FORWARD_SPEED	 140;
#define HARD_TURN_SPEED	 45;
#define LIGHT_TURN_SPEED 75;


// Optimized for going forward. 
// The turns for these values are not unstable, so while going forward it stays smooth
//#define FORWARD_SPEED		160;
//#define HARD_TURN_SPEED	 80;
//#define LIGHT_TURN_SPEED	100;

// Optimized for both directions.
//#define FORWARD_SPEED		120;
//#define HARD_TURN_SPEED	 45;
//#define LIGHT_TURN_SPEED	 75;

#define VREF 5.00; // Vref voltage is 5V

// Different states of the robot.
// Save these states to implement the algorithm: when no sensor is black, go to the direction of last black
#define state_left_hard -2
#define state_left		-1
#define state_straight	 0
#define state_right		 1
#define state_right_hard 2

// Current state and last state
volatile int current_state	= state_straight;
volatile int last_state		= state_straight;	// Save the last state

// Voltages read from the sensors.
// The car has three sensors - left, center, and right
volatile unsigned int read_left;
volatile unsigned int read_right;
volatile unsigned int read_center;
volatile float 		  voltage_left;
volatile float 		  voltage_right;
volatile float 		  voltage_center;

// UART FILE
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

// Initialize all states and functions
void initialize_all(void)
{
	// Set up ADMUX
	ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
	
	// Pre-scalar of 1024
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	
	ADCSRA |= (1<<ADEN);
	ADMUX  |= (1<<REFS0);

	DDRD |= (1<<DDD2) | (1<<DDD4) | (1<<DDD7) | (1<<DDD5) | (1<<DDD6);
	DDRB |= (1<<DDB0);

	PORTD &= ~(1<<PORTD7);
	PORTB |= (1<<PORTB0); 

	PORTD |= (1<<PORTD2);	
	PORTD &= ~(1<<PORTD4);	

	//PWM
	TCCR1A |= (1<<WGM11) | (1<<WGM10);
	TCCR1B |= (1<<WGM13) | (1<<WGM12);					
	
	TCCR1A |= (1<<COM1A1);	

	TCCR1B |= (1<<CS11)  | (1<<CS10);					
	OCR1A	= 249;										
	OCR1B	= 125;                                        
	TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B);

	//TIMER
	TCCR2A |= (1<<WGM21) | (1<<WGM20) | (1<<COM2A1) | (1<<COM2B1);
	TCCR2B |= (1<<WGM22);
	TCCR2B |= (1<<CS21) | (1<<CS22);
	OCR2A	= 249;
	OCR2B	= 125;
	TIMSK2 |= (1<<OCIE2A) | (1<<OCIE2B);

	sei();
	
	// UART
	uart_init(); 
	stdout = stdin = stderr = &uart_str;
}

// Interrupt when TIMER1 COMPA detects the comparison value
ISR(TIMER1_COMPA_vect)
{
	PORTD |= (1<<PORTD6);
}

// Interrupt when TIMER1 COMPB detects the comparison value
ISR(TIMER1_COMPB_vect)
{
	PORTD &= ~((1<<PORTD6));
}

// Interrupt when TIMER2 COMPA detects the comparison value
ISR(TIMER2_COMPA_vect)
{
	PORTD |= (1<<PORTD5);
}

// Interrupt when TIMER2 COMPB detects the comparison value
ISR(TIMER2_COMPB_vect)
{
	PORTD &= ~((1<<PORTD5));
}


// Functions to set a sensor to read
// Chane the ADMUX and set it to the the desired sensor to read that sensor 
void set_ADC_0()
{
	ADMUX &= ~((1<<MUX0) | (1<<MUX1));
}
void set_ADC_1()
{
	ADMUX |= (1<<MUX0);
	ADMUX &= ~(1<<MUX1);
}
void set_ADC_2()
{
	ADMUX |= (1<<MUX1);
	ADMUX &= ~(1<<MUX0);
}

// Start ADC and wait for it to be available
// Run before reading ADC
void start_ADC_and_wait()
{
	ADCSRA |= (1<<ADSC);
	while((ADCSRA & (1<<ADSC)));
}


// Read the sensors

// Left IR Sensor
void read_left_sensor()
{
	read_left	 = (ADCL); 
	read_left	|= (ADCH<<8);
	voltage_left = (float)read_left/1024.00 * VREF;
}

// Right IR Sensor
void read_right_sensor()
{
	read_right	  = (ADCL);
	read_right   |= (ADCH<<8);
	voltage_right = (float)read_right/1024.00 * VREF;
}

// Center IR Sensor
void read_center_sensor()
{
	read_center	   = (ADCL);
	read_center   |= (ADCH<<8);
	voltage_center = (float)read_center/1024.00 * VREF;
}


// Move and turn functions
// OCR1B = Right  Motor
// OCR2B = Left   Motor
// Going forward, both the motors have the same speed
// While turning, one motor has less speed. Hard turning lowers the speed more

// Forward
void move_forward()
{
	OCR1B = FORWARD_SPEED;
	OCR2B = FORWARD_SPEED;
}

// Hard right turn: the right motor is really slow, left is normal
void hard_right_turn()
{
	OCR1B = HARD_TURN_SPEED;
	OCR2B = FORWARD_SPEED;
}

// Hard left turn: the left motor is really slow, right is normal
void hard_left_turn()
{
	OCR1B = FORWARD_SPEED;
	OCR2B = HARD_TURN_SPEED;
}

// Light left turn: the left motor is slow, right is normal
void left_turn()
{
	OCR1B = FORWARD_SPEED;
	OCR2B = LIGHT_TURN_SPEED;
}

// Light right turn: the right motor is slow, left is normal
void right_turn()
{
	OCR1B = LIGHT_TURN_SPEED;
	OCR2B = FORWARD_SPEED;
}

// Main function
int main(void)
{
	// Initialize everything
	initialize_all();
	
	while (1)
	{
		// Read left sensor
		set_ADC_0();			// Set ADC to read the left sensor
		start_ADC_and_wait();	// Start and wait
		read_left_sensor();		// Read the value

		set_ADC_1();			// Set ADC to read the right sensor
		start_ADC_and_wait();	// Start and wait
		read_right_sensor();	// Read the value

		set_ADC_2();			// Set ADC to read the middle sensor
		start_ADC_and_wait();	// Start and wait
		read_center_sensor();	// Read the value

		//----------------------------------------------------------------------------------------
		// LOGIC IMPLEMENTATION
		//----------------------------------------------------------------------------------------
		
		// If left and right sensors are black: go straight
		// If center sensor is black and left and right are not: go straight
		if ( (voltage_left >= BLACK_THRESHOLD && voltage_right >= BLACK_THRESHOLD) ||
			((voltage_center >= BLACK_THRESHOLD) && (voltage_left < BLACK_THRESHOLD && voltage_right < BLACK_THRESHOLD)) )
		{
			move_forward();
			current_state = state_straight;
		}
		// If left sensor is black: go left
		else if (voltage_left >= BLACK_THRESHOLD) 
		{
			// If the left sensor is black and center sensor is also black, lightly turn left.
			// Else, hard turn left
			if (voltage_center >= BLACK_THRESHOLD)
			{
				left_turn();
				current_state = state_left;
			}
			else
			{
				hard_left_turn();
				current_state = state_left_hard;
			}
		}
		// If the right sensor is black: go right
		else if (voltage_right >= BLACK_THRESHOLD)
		{
			// If the right sensor is black and center sensor is also black, lightly turn right.
			// Else, hard turn right
			if (voltage_center >= BLACK_THRESHOLD)
			{
				right_turn();
				current_state = state_right;
			}
			else
			{
				hard_right_turn();
				current_state = state_right_hard;
			}
		}
		// No sensors are black
		// Keep going in the direction of last sensor that was detected as black
		else
		{
			if (last_state == state_right || last_state == state_right_hard)
			{
				hard_right_turn();
				current_state = state_right;
			}
			else if (last_state == state_left || last_state == state_left_hard)
			{
				hard_left_turn();
				current_state = state_left;
			}
			else
			{
				move_forward();
				last_state = state_straight;
			}
		}
	
		// Save state
		last_state = current_state;
	}
}
