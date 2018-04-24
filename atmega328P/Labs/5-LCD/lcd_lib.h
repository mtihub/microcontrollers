// mixed 8 bit LCD interface for ECE-3411
// By Marten van Dijk, Jan 2014:
// Derived from (1) Prof. Sung-Yuel Park's ECE-3411 lectures, and (2):

// File Name	: 'lcd_lib.c'
// Title		: 8 and 4 bit LCd interface
// Author		: Scienceprog.com - Copyright (C) 2007
// Created		: 2007-03-29
// Revised		: 2007-08-08
// Version		: 1.0
// Target MCU	: Atmel AVR series
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt

#include <inttypes.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define LCD_DDRAM           	7	//DB7: set DD RAM address

#define LCD_LINE0_DDRAMADDR		0x00
#define LCD_LINE1_DDRAMADDR		0x40


#define DATA_PORT				PORTC
#define DATA_PORT_DDR			DDRC
#define CTRL_PORT				PORTC
#define CTRL_PORT_DDR			DDRC
#define RS						4
//#define RW					1	// Always tied to GND for now
#define ENABLE					5

void LDCPortConfig(void); //Configuration ports (used in initialize_LCD)
void LcdCommandWrite(uint8_t);	//forms data ready to send to LCD
void LcdCommandWrite_UpperNibble(uint8_t);	//forms data ready to send to LCD
void LcdDataWrite(uint8_t);		//forms data ready to send to LCD
void DisableJTAG(void); //Disable JTAG (used in initialize_LCD)
void initialize_LCD(void); //Initializes LCD
void LCDclr(void);	//Clears LCD
void LCDhome(void); //LCD cursor home
void LCDstring(uint8_t*, uint8_t);	//Outputs string to LCD
void LCDGotoXY(uint8_t, uint8_t);	//Cursor to X Y position
void CopyStringtoLCD(const uint8_t*, uint8_t, uint8_t); //copies flash string to LCD at x,y
void LCDshiftRight(uint8_t);	//shift by n characters Right
void LCDshiftLeft(uint8_t);	    //shift by n characters Left
void LCDcursorOn(void);		    //Underline cursor ON
void LCDcursorOnBlink(void);	//Underline blinking cursor ON
void LCDcursorOFF(void);		//Cursor OFF
void LCDblank(void);			//LCD blank but not cleared
void LCDvisible(void);			//LCD visible
void LCDcursorLeft(uint8_t);	//Shift cursor left by n
void LCDcursorRight(uint8_t);	//shif cursor right by n