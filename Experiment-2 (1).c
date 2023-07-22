//---------------------------------- HEADER FILES -----------------------------------------------------
#include "firebird_simulation.h"		// Header file included that contains macro definitions essential for Firebird V robot
#include <stdbool.h>					// Standard C Library for Boolean Type

#include <stdio.h>						// Standard C Library for standard input output
#include "lcd.h"						// LCD Header file included that contains function definitions essential to deal with LCD
#include "uart.h"						// UART Header file included that contains function definitions essential to deal with UART


//------------------------------------- MACROS ----------------------------------------------------------
// If any
#define sharp_sensor_pin_num 2
//---------------------------------- GLOBAL VARIABLES ---------------------------------------------------
// If any
unsigned int adc_10bit_data;						// To store 10-bit ADC converted data of a sensor

//---------------------------------- FUNCTIONS ----------------------------------------------------------
// << TODO >> : Complete all the functions as per the instructions given in form of comments


/**
 * @brief      Makes **ONLY** 2nd Sharp sensor pin as input and deactivates pull-up for **ONLY** 2nd Sharp sensor pin
 */
#define sharp_sensor_pin_num 2
void sharp_sensor_pin_config(){
	// << NOTE >> : Use Masking and Shift Operators here
	
	// Make **ONLY** 2nd Sharp sensor pin as input
	sharp_sensor_ddr_reg &= ~(1 <<  sharp_sensor_pin_num);
	
	// Deactivate pull-up for **ONLY** 2nd Sharp sensor pin
	sharp_sensor_port_reg &= ~(1 << sharp_sensor_pin_num);
}


/**
 * @brief      Initializes the Analog-to-Digital converter inside the micro-controller
 */
void adc_init(){
	// << NOTE >> : Use Masking and Shift Operators here
	
	// Clear the Global Interrupt Flag bit in SREG
	SREG &= ~(1 << 7);

	// In ADCSRA, enable ADC, ADC Interrupt and pre-scalar = 64 
	//				and clear ADC start conversion bit, auto trigger enable bit, interrupt flag bit and interrupt enable bit
	ADCSRA_reg = (1 << ADEN_bit) | (1 << ADIE_bit) | (1 << ADPS2_bit) | (1 << ADPS1_bit);

	// In ADCSRB, disable Analog Comparator Multiplexer, MUX5 bit and ADC Auto Trigger Source bits
	ADCSRB_reg = 0x00;

	// In ADMUX, set the Reference Selection bits to use the AVCC as reference, and clear the channel selection bits MUX[4:0] initially
	ADMUX_reg = (1 << REFS0_bit);

	// In ADMUX, disable or enable the ADLAR bit to read 10-bit ADC result based on your logic
	ADMUX_reg |= (1 << ADLAR_bit);

	// In ACSR, disable the Analog Comparator by writing 1 to ACD_bit
	ACSR_reg |= (1 << ACD_bit);

	// Set the Global Interrupt Flag bit in SREG
	SREG |= (1 << 7);
}


/**
 * @brief      Sets the MUX[5:0] bits according to the sensor's channel number as input
 */
void select_adc_channel( unsigned char channel_num ){
	// << NOTE >> : Use Masking and Shift Operators here
	
	// set the MUX[5:0] bits to select the ADC channel number
	ADMUX_reg = (ADMUX_reg & 0xE0) | (channel_num & 0x1F);
}


/**
 * @brief      Starts the ADC by setting the ADSC bit in ADCSRA register
 */
void start_adc(void){
	// << NOTE >> : Use Masking and Shift Operators here
	
	// set the ADSC bit in ADCSRA register
	ADCSRA_reg |= (1 << ADSC_bit);
}


/**
 * @brief      Reset ADC config registers, ADCSRB and ADMUX registers
 */
void reset_adc_config_registers(void){
	// << NOTE >> : Use Masking and Shift Operators here

	// clear the MUX5 bit for next conversion
	ADCSRB_reg &= ~(1 << MUX5_bit);

	// clear the MUX[4:0] bits for next conversion
	ADMUX_reg &= 0xE0;
}


// read the ADC data after the conversion is complete
ISR( ADC_vect ){

	adc_10bit_data = ADC;

}


/**
 * @brief      Convert the analog readings to 10-bit digital format from the sensor's ADC channel number as input
 */
void convert_analog_channel_data( unsigned char sensor_channel_number ){
	
	// << NOTE >> : You are not allowed to modify or change anything inside this function
	
	select_adc_channel( sensor_channel_number );
	
	start_adc();
}


//---------------------------------- MAIN ----------------------------------------------------------------
/**
 * @brief      Main Function
 *
 * @details    First Initializes the 2nd Sharp sensor and sends its ADC converted data on LCD and UART
 */
int main(void) {
	
	// << NOTE >> : You are not allowed to modify or change anything inside this function except a part of while loop
	
	
	sharp_sensor_pin_config();			// Initialize the 2nd Sharp sensor
	
	adc_init();							// Initialize the ADC
	
	lcd_port_config();					// Initialize the LCD port
	lcd_init();							// Initialize the LCD
	
	uart_init(UBRR_VALUE);				// Initialize the UART
	
	// To store 10-bit data of 2nd Sharp sensor
	int sharp_sensor_data;
	
	// To store the pin number of 2nd Sharp sensor
	unsigned char sharp_sensor_pin_num = 2;
	
	// To create string for transmitting 2nd Sharp sensor raw data and corresponding voltage over the UART and LCD
	char tx_raw_adc_data_buffer[25], tx_voltage_buffer[25], lcd_print_raw_adc_data_string[25], lcd_print_voltage_string[25];
	
	convert_analog_channel_data( sharp_sensor_channel );
	
	while(1)
	{
		if ( ( ADCSRA_reg & ( 1 << ADSC_bit ) ) != 0x40 )
		{
			// ------------------------------- //

			sharp_sensor_data = adc_10bit_data;


			// ------------------------------- //
			
			sprintf(lcd_print_raw_adc_data_string, "Raw data: %04d", sharp_sensor_data);
			lcd_string(1, 1, lcd_print_raw_adc_data_string);
			
			sprintf(tx_raw_adc_data_buffer, "Raw data: %04d\t", sharp_sensor_data);
			uart_tx_string(tx_raw_adc_data_buffer);
			
			convert_analog_channel_data( sharp_sensor_channel );
		}
	}
}
//---------------------------------- END ------------------------------------------------------------------

