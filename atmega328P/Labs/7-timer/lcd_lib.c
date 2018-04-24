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

  
#include "lcd_lib.h"
#include <inttypes.h> 
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
 
void LDCPortConfig(void)
{
	// LDC port configuration for the MCU
	DATA_PORT_DDR |= 0x0f;  							//	set DATA_PORT[3:0] as output (4-bit mode)
	//CTRL_PORT_DDR |= ((1<<RS)|(1<<RW)|(1<<ENABLE));  	//	Set control port's respective signals as output
	CTRL_PORT_DDR |= ((1<<RS)|(1<<ENABLE));  	//	Set control port's respective signals as output
	//CTRL_PORT &= ~(1<<RW); 								// set the RW as writing (logic low) for always writing
}

void LcdCommandWrite_UpperNibble(uint8_t cm)
{
	DATA_PORT = (DATA_PORT & 0xf0) | (cm >> 4); //give the higher half of cm to DATA_PORT
	CTRL_PORT &= ~(1<<RS);		//setting RS=0 to choose the instruction register
	CTRL_PORT |= (1<<ENABLE);	//setting ENABLE=1
	_delay_ms(1); 	// allow the LCD controller to successfully read command in, minimum 37 µs
	CTRL_PORT &= ~(1<<ENABLE); // Setting ENABLE=0
	_delay_ms(1);  // allow long enough delay for instruction writing
}

void LcdCommandWrite(uint8_t cm)
{
	// First send higher 4-bits
	DATA_PORT = (DATA_PORT & 0xf0) | (cm >> 4); //give the higher half of cm to DATA_PORT
	CTRL_PORT &= ~(1<<RS);		//setting RS=0 to choose the instruction register
	CTRL_PORT |= (1<<ENABLE);	//setting ENABLE=1
	_delay_ms(1); 	// allow the LCD controller to successfully read command in, minimum 37 µs
	CTRL_PORT &= ~(1<<ENABLE); // Setting ENABLE=0
	_delay_ms(2);  // allow long enough delay for instruction writing
	
	// Send lower 4-bits
	DATA_PORT = (DATA_PORT & 0xf0) | (cm & 0x0f); //give the lower half of cm to DATA_PORT
	CTRL_PORT &= ~(1<<RS);		//setting RS=0 to choose the instruction register
	CTRL_PORT |= (1<<ENABLE);	//setting ENABLE=1
	_delay_ms(1); 	// allow the LCD controller to successfully read command in, minimum 37 µs
	CTRL_PORT &= ~(1<<ENABLE); // Setting ENABLE=0
	_delay_ms(2);  // allow long enough delay for instruction writing
}
 
void LcdDataWrite(uint8_t da)
{
	// First send higher 4-bits
	DATA_PORT = (DATA_PORT & 0xf0) | (da >> 4); //give the higher half of cm to DATA_PORT
	CTRL_PORT |= (1<<RS);		//setting RS=1 to choose the data register
	CTRL_PORT |= (1<<ENABLE);	//setting ENABLE=1
	_delay_ms(1); 	// allow the LCD controller to successfully read command in, minimum 37 µs
	CTRL_PORT &= ~(1<<ENABLE); // Setting ENABLE=0
	_delay_ms(1);  // allow long enough delay 
	
	// Send lower 4-bits
	DATA_PORT = (DATA_PORT & 0xf0) | (da & 0x0f); //give the lower half of cm to DATA_PORT
	CTRL_PORT |= (1<<RS);		//setting RS=1 to choose the data register
	CTRL_PORT |= (1<<ENABLE);	//setting ENABLE=1
	_delay_ms(1); 	// allow the LCD controller to successfully read command in, minimum 37 µs
	CTRL_PORT &= ~(1<<ENABLE); // Setting ENABLE=0
	_delay_ms(2);  // allow long enough delay 

}
 
void DisableJTAG(void)
{
	//disable JTAG
	//unsigned char temp;
	//temp = MCUCR;
	//MCUCR = temp | (1<<JTD);
	//MCUCR = temp | (1<<JTD);
}
 
void initialize_LCD(void)
{
	//DisableJTAG();
	LDCPortConfig();
	
	
	// initialize LCD
	LcdCommandWrite_UpperNibble(0x30);   // function set: 8-bit interface
	_delay_ms(4.1);
	LcdCommandWrite_UpperNibble(0x30);
	_delay_us(100);
	LcdCommandWrite_UpperNibble(0x30);
	//_delay_ms(4.1);
	LcdCommandWrite_UpperNibble(0x20);
	
	//LcdCommandWrite(0x38);   // function set: 0x38 means,  8-bit interface, 2 lines, 5x8 font
	LcdCommandWrite(0x28);   // function set: 0x28 means,  4-bit interface, 2 lines, 5x8 font
	LcdCommandWrite(0x08);   // display control: turn display off, cursor off, no blinking
	LcdCommandWrite(0x01);   // clear display, set address counter  to zero
	LcdCommandWrite(0x06);   // entry mode set:
	
	//   cursor increments automatically, no display shift
	LcdCommandWrite(0x0f);   // display on, cursor on, cursor blinking
	_delay_ms(120);
	
}

void LCDclr(void) //Clears LCD
{
	LcdCommandWrite(0x01);
}

void LCDhome(void) //LCD cursor home
{
	LcdCommandWrite(0x02);
}

void LCDstring(uint8_t* data, uint8_t nBytes)	//Outputs string to LCD
{
	register uint8_t i;

	// check to make sure we have a good pointer
	if (!data) return;

	// print data
	for(i=0; i<nBytes; i++)
	{
		LcdDataWrite(data[i]);
	}
}

void LCDGotoXY(uint8_t x, uint8_t y) //Cursor to X Y position
{
	register uint8_t DDRAMAddr;
	// remap lines into proper order
	switch(y)
	{
		case 0: DDRAMAddr = LCD_LINE0_DDRAMADDR+x; break;
		case 1: DDRAMAddr = LCD_LINE1_DDRAMADDR+x; break;
		default: DDRAMAddr = LCD_LINE0_DDRAMADDR+x;
	}
	// set data address
	LcdCommandWrite(1<<LCD_DDRAM | DDRAMAddr);	
}

//Copies string from flash memory to LCD at x y position
//const uint8_t welcomeln1[] PROGMEM="AVR LCD DEMO\0";
//CopyStringtoLCD(welcomeln1, 3, 1);
void CopyStringtoLCD(const uint8_t *FlashLoc, uint8_t x, uint8_t y)
{
	uint8_t i;
	LCDGotoXY(x,y);
	for(i=0;(uint8_t)pgm_read_byte(&FlashLoc[i]);i++)
	{
		LcdDataWrite((uint8_t)pgm_read_byte(&FlashLoc[i]));
	}
}

void LCDshiftLeft(uint8_t n)	//Scrol n of characters Right
{
	for (uint8_t i=0;i<n;i++)
	{
		LcdCommandWrite(0x1E);
	}
}

void LCDshiftRight(uint8_t n)	//Scrol n of characters Left
{
	for (uint8_t i=0;i<n;i++)
	{
		LcdCommandWrite(0x18);
	}
}

void LCDcursorOn(void) //displays LCD cursor
{
	LcdCommandWrite(0x0E);
}

void LCDcursorOnBlink(void)	//displays LCD blinking cursor
{
	LcdCommandWrite(0x0F);
}

void LCDcursorOFF(void)	//turns OFF cursor
{
	LcdCommandWrite(0x0C);
}

void LCDblank(void)		//blanks LCD
{
	LcdCommandWrite(0x08);
}

void LCDvisible(void)		//Shows LCD
{
	LcdCommandWrite(0x0C);
}

void LCDcursorLeft(uint8_t n)	//Moves cursor by n positions left
{
	for (uint8_t i=0;i<n;i++)
	{
		LcdCommandWrite(0x10);
	}
}

void LCDcursorRight(uint8_t n)	//Moves cursor by n positions left
{
	for (uint8_t i=0;i<n;i++)
	{
		LcdCommandWrite(0x14);
	}
}
