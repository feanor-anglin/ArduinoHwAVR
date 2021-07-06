/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/
  Copyright (c) 2007 David A. Mellis
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// Digital pins
#define NUM_DIGITAL_PINS            27
//#define NUM_ANALOG_INPUTS           6
//#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : -1)

// PWM pins
#define digitalPinHasPWM(p)         ((p) == 0 || (p) == 1 || (p) == 2 ||(p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)

// Builtin LEDs
#define LED_BUILTIN_1 (23)
#define LED_BUILTIN_2 (24)
static const uint8_t LED_1 = LED_BUILTIN_1;
static const uint8_t LED_2 = LED_BUILTIN_2;

// Analog pins
#define PIN_A0 (14)
#define PIN_A1 (15)
#define PIN_A2 (16)
#define PIN_A3 (17)
#define PIN_A4 (18)
#define PIN_A5 (19)
#define PIN_A6 (25)
#define PIN_A7 (26)
static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;
#define NUM_ANALOG_INPUTS           (8)
#define analogInputToDigitalPin(p)  (((p) < 6) ? ((p) + 14) : (((p) < 8) ? ((p) + 19) : -1))
#define analogPinToChannel(p)       ((p) < NUM_ANALOG_INPUTS ? (p) : ((p) >= 14 && (p) < 25) ? (p) - 14 : ((p) >= 25) ? (p) - 19 : -1)

// SPI
#define PIN_SPI_SS0   (10)
#define PIN_SPI_MOSI0 (11)
#define PIN_SPI_MISO0 (12)
#define PIN_SPI_SCK0  (13)
#define PIN_SPI_SS1   (25)
#define PIN_SPI_MOSI1 (26)
#define PIN_SPI_MISO1 (14)
#define PIN_SPI_SCK1  (15)
static const uint8_t SS0   = PIN_SPI_SS0;
static const uint8_t MOSI0 = PIN_SPI_MOSI0;
static const uint8_t MISO0 = PIN_SPI_MISO0;
static const uint8_t SCK0  = PIN_SPI_SCK0;
static const uint8_t SS1   = PIN_SPI_SS1;
static const uint8_t MOSI1 = PIN_SPI_MOSI1;
static const uint8_t MISO1 = PIN_SPI_MISO1;
static const uint8_t SCK1  = PIN_SPI_SCK1;

// I2C
#define WIRE_INTERFACES_COUNT (2)
#define PIN_WIRE_SDA0 (18)
#define PIN_WIRE_SCL0 (19)
#define PIN_WIRE_SDA1 (23)
#define PIN_WIRE_SCL1 (24)
static const uint8_t SDA0  = PIN_WIRE_SDA0;
static const uint8_t SCL0  = PIN_WIRE_SCL0;
static const uint8_t SDA1  = PIN_WIRE_SDA1;
static const uint8_t SCL1  = PIN_WIRE_SCL1;

// Interrupts
#define EXTERNAL_NUM_INTERRUPTS     (2)
#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))

// PCINT
#define digitalPinToPCICR(p)     (((p) >= 0 && (p) <= 26) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p)  (((p) <= 7) ? 2 : (((p) <= 13 || (p) == 20 || (p) == 21) ? 0 : (((p) <= 22) ? 1 : 3)))
#define digitalPinToPCMSK(p)     (((p) <= 7) ? (&PCMSK2) : (((p) <= 13 || (p) == 20 || (p) == 21) ? (&PCMSK0) : (((p) <= 22) ? (&PCMSK1) : (((p) <= 26) ? (&PCMSK3) : ((uint8_t *)0)))))
#define digitalPinToPCMSKbit(p)  (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) <= 21) ? ((p) - 14) : ((p) == 22) ? ((p) - 16) : ((p) - 23)))

#define PIN_PD0 0
#define PIN_PD1 1
#define PIN_PD2 2
#define PIN_PD3 3
#define PIN_PD4 4
#define PIN_PD5 5
#define PIN_PD6 6
#define PIN_PD7 7
#define PIN_PB0 8
#define PIN_PB1 9
#define PIN_PB2 10
#define PIN_PB3 11
#define PIN_PB4 12
#define PIN_PB5 13
#define PIN_PC0 14 // A0
#define PIN_PC1 15 // A1
#define PIN_PC2 16 // A2
#define PIN_PC3 17 // A3
#define PIN_PC4 18 // A4
#define PIN_PC5 19 // A5
#define PIN_PB6 20 // XTAL1
#define PIN_PB7 21 // XTAL2
#define PIN_PC6 22 // RESET
#define PIN_PE0 23
#define PIN_PE1 24
#define PIN_PE2 25 // A6
#define PIN_PE3 26 // A7

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+
//
// (PWM+ indicates the additional PWM pins on the ATmega168.)

// ATMEL ATMEGA1280 / ARDUINO
//
// 0-7 PE0-PE7   works
// 8-13 PB0-PB5  works
// 14-21 PA0-PA7 works 
// 22-29 PH0-PH7 works
// 30-35 PG5-PG0 works
// 36-43 PC7-PC0 works
// 44-51 PJ7-PJ0 works
// 52-59 PL7-PL0 works
// 60-67 PD7-PD0 works
// A0-A7 PF0-PF7
// A8-A15 PK0-PK7


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD, // PD0 - D0
	PD, // PD1 - D1
	PD, // PD2 - D2
	PD, // PD3 - D3
	PD, // PD4 - D4
	PD, // PD5 - D5
	PD, // PD6 - D6
	PD, // PD7 - D7
	PB, // PB0 - D8
	PB, // PB1 - D9
	PB, // PB2 - D10
	PB, // PB3 - D11
	PB, // PB4 - D12
	PB, // PB5 - D13
	PC, // PC0 - D14 / A0
	PC, // PC1 - D15 / A1
	PC, // PC2 - D16 / A2
	PC, // PC3 - D17 / A3
	PC, // PC4 - D18 / A4
	PC, // PC5 - D19 / A5
	PB, // PB6 - D20 / XTAL1
	PB, // PB7 - D21 / XTAL2
	PC, // PC6 - D22 / RESET
	PE, // PE0 - D23
	PE, // PE1 - D24
	PE, // PE2 - D25 / A6
	PE, // PE3 - D26 / A7
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), // PD0 - D0
	_BV(1), // PD1 - D1
	_BV(2), // PD2 - D2
	_BV(3), // PD3 - D3
	_BV(4), // PD4 - D4
	_BV(5), // PD5 - D5
	_BV(6), // PD6 - D6
	_BV(7), // PD7 - D7
	_BV(0), // PB0 - D8
	_BV(1), // PB1 - D9
	_BV(2), // PB2 - D10
	_BV(3), // PB3 - D11
	_BV(4), // PB4 - D12
	_BV(5), // PB5 - D13
	_BV(0), // PC0 - D14 / A0
	_BV(1), // PC1 - D15 / A1
	_BV(2), // PC2 - D16 / A2
	_BV(3), // PC3 - D17 / A3
	_BV(4), // PC4 - D18 / A4
	_BV(5), // PC5 - D19 / A5
	_BV(6), // PB6 - D20 / XTAL1
	_BV(7), // PB7 - D21 / XTAL2
	_BV(6), // PC6 - D22 / RESET
	_BV(0), // PE0 - D23
	_BV(1), // PE1 - D24
	_BV(2), // PE2 - D25 / A6
	_BV(3), // PE3 - D26 / A7
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	TIMER3A,      // PD0 - D0
	TIMER4A,      // PD1 - D1
	TIMER4B,      // PD2 - D2 -> TIMER3B is also an option
	TIMER2B,      // PD3 - D3
	NOT_ON_TIMER, // PD4 - D4
	TIMER0B,      // PD5 - D5
	TIMER0A,      // PD6 - D6
	NOT_ON_TIMER, // PD7 - D7
	NOT_ON_TIMER, // PB0 - D8
	TIMER1A,      // PB1 - D9
	TIMER1B,      // PB2 - D10
	TIMER2A,      // PB3 - D11
	NOT_ON_TIMER, // PB4 - D12
	NOT_ON_TIMER, // PB5 - D13
	NOT_ON_TIMER, // PC0 - D14 / A0
	NOT_ON_TIMER, // PC1 - D15 / A1
	NOT_ON_TIMER, // PC2 - D16 / A2
	NOT_ON_TIMER, // PC3 - D17 / A3
	NOT_ON_TIMER, // PC4 - D18 / A4
	NOT_ON_TIMER, // PC5 - D19 / A5
	NOT_ON_TIMER, // PB6 - D20 / XTAL1
	NOT_ON_TIMER, // PB7 - D21 / XTAL2
	NOT_ON_TIMER, // PC6 - D22 / RESET
	NOT_ON_TIMER, // PE0 - D23
	NOT_ON_TIMER, // PE1 - D24
	NOT_ON_TIMER, // PE2 - D25 / A6
	NOT_ON_TIMER, // PE3 - D26 / A7
};

#endif

// Make sure the ATmega328PB is backwards compatible with the 328 and 328P
#if defined(__AVR_ATmega328PB__)
// SPI
  #define SPCR SPCR0
  #define SPSR SPSR0
  #define SPDR SPDR0
  #define SPI_STC_vect SPI0_STC_vect
  #define SPI_STC_vect_num SPI0_STC_vect_num
// I2C
  #define TWBR TWBR0
  #define TWSR TWSR0
  #define TWAR TWAR0
  #define TWDR TWDR0
  #define TWCR TWCR0
  #define TWAMR TWAMR0
  #define TWI_vect TWI0_vect
  #define TWI_vect_num TWI0_vect_num
// UART
  #define USART_RX_vect USART0_RX_vect
  #define USART_RX_vect_num USART0_RX_vect_num
  #define USART_UDRE_vect USART0_UDRE_vect
  #define USART_UDRE_vect_num USART0_UDRE_vect_num
  #define USART_TX_vect USART0_TX_vect
  #define USART_TX_vect_num USART0_TX_vect_num
#endif // 328PB defs

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR   		Serial
#define SERIAL_PORT_HARDWARE  		Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial

#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial1

#endif