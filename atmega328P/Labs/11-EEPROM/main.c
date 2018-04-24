/*
 * Lab12EEPROMUsernamePassword.c
 *
 * Created: 3/5/2018 4:19:13 PM
 * Author : Tanvir-Laptop
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "lcd_lib.h"
#include "uart.h"
#include <avr/eeprom.h>
#include <avr/wdt.h>

// EEPROM write addresses
#define eeprom_true 0
#define eeprom_data 1
volatile unsigned int Ain;
volatile float Voltage;
volatile uint32_t counter;
volatile uint32_t time;
const int8_t LCD_initialize[] PROGMEM = "LCD Initialized";
const int8_t LCD_voltage[] PROGMEM = "Volts ";
const int8_t LCD_counter[] PROGMEM = "Count ";
int8_t lcd_buffer[17]; // LCD display buffer

// File stream for UART. Used for Transmission to demonstrate the fprintf function.
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);


void write_to_eeprom(char* str) {
	eeprom_write_dword((uint8_t*)eeprom_data, str);			//Write our current value to EEPROM
	eeprom_write_byte((uint8_t*)eeprom_true, 'T');			//Set write flag TRUE
}

char* read_from_eeprom() {
	if (eeprom_read_byte((uint8_t*)eeprom_true) == 'T') // if a value was stored at all.
	{
		char* written_data = eeprom_read_dword((uint8_t*)eeprom_data);
		return written_data;
	}
	else
	{
		
	}
}


int main() {
	//sei(); // Enable global interrupts
	
	uart_init();                         // Initialize UART
	stdout = stdin = stderr = &uart_str; // Set File outputs to point to UART stream
	printf("\n\nInitialize the credentials... \n");
	
	// Take Username input:
	char username[20];
	printf("Enter Username: ");
	scanf("%s", &username);
	
	// Take Password input:
	char password[20];
	printf("Enter Password: ");
	scanf("%s", &password);
	
	//char username[20] = "username";
	//char password[20] = "password";
	
	// Write data to eeprom
	char credentials[42];
	strcpy(credentials, username);
	strcat(credentials, " ");
	strcat(credentials, password);
	write_to_eeprom(credentials);
	
	// Login
	printf("\n\nLogin System Initialized.\n");
	int check_again = 1;
	while (check_again) {
		int is_username_correct = 1;
		int is_password_correct = 1;
		
		printf("\nEnter username and password.\n");
		char *eeprom_credentials = read_from_eeprom();
		//printf("\nCredentials Read from EEPROM: %s\n\n", eeprom_credentials);
		char check_username[20];
		char check_password[20];
		
		int i = 0, j = 0;
		while (eeprom_credentials[i] != ' ') {
			check_username[j] = eeprom_credentials[i];
			i++;
			j++;
		}
		
		i++;
		j = 0;
		while (eeprom_credentials[i] != '\0') {
			check_password[j] = eeprom_credentials[i];
			i++;
			j++;
		}
		
		char username_input[20];
		char password_input[20];
		printf("Username: ");
		scanf("%s", &username_input);
		printf("Password: ");
		scanf("%s", &password_input);
		
		int original_username_len = strlen(check_username);
		int original_password_len = strlen(check_password);
		int input_username_len    = strlen(username_input);
		int input_password_len    = strlen(password_input);
		
		//printf("\nChecking Username\n");
		i = 0;
		while (username_input[i] != '\0') {
			//if (input_username_len != original_username_len) {
			//	is_username_correct = 0;
			//	break;
			//}
			if (check_username[i] == '\0') {
				is_username_correct = 0;
				break;
			}
			if (username_input[i] != check_username[i]) {
				is_username_correct = 0;
				break;
			}
			else {
				i++;
				continue;
			}
		}
		
		//printf("\nChecking Password\n");
		i = 0;
		while (password_input[i] != '\0') {
			if (check_password[i] == '\0') {
				is_password_correct = 0;
				break;
			}
			if (password_input[i] != check_password[i]) {
				is_password_correct = 0;
				break;
			}
			else {
				i++;
				continue;
			}
		}
		if (password_input[i] == '\0' && check_password[i] != '\0') {
			is_password_correct = 0;
		}
		
		if (is_username_correct == 1 && is_password_correct == 1) {
			check_again = 0;
		}
	}
	
	printf("\n\nUsername and Password Accepted!\n");
}
