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
*/
/*
 * Modified 13 July 2014 by Nozomu Fujita
 * Modified  2 Mar  2017 by Yuuki Okamiya
 */

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#ifndef __RL78__
#include <avr/pgmspace.h>
#else
#include <stdint.h>
#include <stdlib.h>
#endif

#ifndef __RL78__
#define NUM_DIGITAL_PINS            20
#define NUM_ANALOG_INPUTS           6
#else
#define NUM_DIGITAL_PINS            31
#define NUM_ANALOG_INPUTS           8
#define NUM_SWPWM_PINS              4
#endif
#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : -1)

#if defined(__AVR_ATmega8__)
#define digitalPinHasPWM(p)         ((p) == 9 || (p) == 10 || (p) == 11)
#elif defined(__RL78__)
#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11|| (p) == 22|| (p) == 23|| (p) == 24)
#else
#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11)
#endif

#define PIN_SPI_SS    (10)
#define PIN_SPI_MOSI  (11)
#define PIN_SPI_MISO  (12)
#define PIN_SPI_SCK   (13)
#ifdef __RL78__
#define PIN_SPI2_SS    (27)
#define PIN_SPI2_MOSI  (28)
#define PIN_SPI2_MISO  (29)
#define PIN_SPI2_SCK   (30)
#endif

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;
#ifdef __RL78__
static const uint8_t SS2   = PIN_SPI2_SS;
static const uint8_t MOSI2 = PIN_SPI2_MOSI;
static const uint8_t MISO2 = PIN_SPI2_MISO;
static const uint8_t SCK2  = PIN_SPI2_SCK;
#endif

#ifndef __RL78__
#define PIN_WIRE_SDA        (18)
#define PIN_WIRE_SCL        (19)
#else
#define PIN_WIRE_SDA        (8)
#define PIN_WIRE_SCL        (7)
#endif /*__RL78__*/

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#ifdef GRADZUKI
#define LED_BUILTIN 13
#else
#define LED_BUILTIN 24
#endif

#define PIN_A0   (14)
#define PIN_A1   (15)
#define PIN_A2   (16)
#define PIN_A3   (17)
#define PIN_A4   (18)
#define PIN_A5   (19)
#define PIN_A6   (20)
#define PIN_A7   (21)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) <= 21) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))
#define digitalPinToPCMSK(p)    (((p) <= 7) ? (&PCMSK2) : (((p) <= 13) ? (&PCMSK0) : (((p) <= 21) ? (&PCMSK1) : ((uint8_t *)0))))
#define digitalPinToPCMSKbit(p) (((p) <= 7) ? (p) : (((p) <= 13) ? ((p) - 8) : ((p) - 14)))

#ifndef __RL78__
#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : NOT_AN_INTERRUPT))
#else
#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : (p) == 25 ? 2 : NOT_AN_INTERRUPT))
#endif

#ifdef __RL78__
/***************************************************************************/
/*    Macro Definitions                                                    */
/***************************************************************************/
#define ADDR_PORT_REG			0xFFFFFF00
#define ADDR_PORT_MODE_REG		0xFFFFFF20
#define ADDR_PORT_PULL_UP_REG	0xFFFF0030
#define	ADDR_PORT_IN_MODE_REG	0xFFFF0040
#define ADDR_PORT_OUT_MODE_REG	0xFFFF0050

#define ANALOG_PIN_0		2		// ANI2
#define ANALOG_PIN_1		3		// ANI3
#define ANALOG_PIN_2		4		// ANI4
#define ANALOG_PIN_3		5		// ANI5
#define ANALOG_PIN_4		6		// ANI6
#define ANALOG_PIN_5		7		// ANI7
#define ANALOG_PIN_6		18		// ANI18
#define ANALOG_PIN_7		19		// ANI19

/* Port define. */
#define PORT_0				0
#define PORT_1				1
#define PORT_2				2
#define PORT_3				3
#define PORT_4				4
#define PORT_5				5
#define PORT_6				6
#define PORT_7				7
#define PORT_8				8
#define PORT_9				9
#define PORT_10				10
#define PORT_11				11
#define PORT_12				12
#define PORT_13				13
#define PORT_14				14
#define PORT_15				15

/* Port of digital pin define. */
#define DIGITAL_PIN_0		PORT_1	// P11
#define DIGITAL_PIN_1		PORT_1	// P12
#define DIGITAL_PIN_2		PORT_3	// P30
#define DIGITAL_PIN_3		PORT_1	// P16
#define DIGITAL_PIN_4		PORT_3	// P31
#define DIGITAL_PIN_5		PORT_1	// P15
#define DIGITAL_PIN_6		PORT_1	// P10
#ifndef GRCOTTON
#define DIGITAL_PIN_7		PORT_0	// P00
#define DIGITAL_PIN_8		PORT_0	// P01
#else
#define DIGITAL_PIN_7		PORT_0	// P02
#define DIGITAL_PIN_8		PORT_0	// P03
#endif // GRCOTTON
#define DIGITAL_PIN_9		PORT_1	// P13
#define DIGITAL_PIN_10		PORT_1	// P14
#define DIGITAL_PIN_11		PORT_7	// P72
#define DIGITAL_PIN_12		PORT_7	// P71
#define DIGITAL_PIN_13		PORT_7	// P70
#define DIGITAL_PIN_14		PORT_2	// P22
#define DIGITAL_PIN_15		PORT_2	// P23
#define DIGITAL_PIN_16		PORT_2	// P24
#define DIGITAL_PIN_17		PORT_2	// P25
#define DIGITAL_PIN_18		PORT_2	// P26
#define DIGITAL_PIN_19		PORT_2	// P27
#define DIGITAL_PIN_20		PORT_14	// P147
#define DIGITAL_PIN_21		PORT_12	// P120
#define DIGITAL_PIN_22		PORT_1	// P17
#define DIGITAL_PIN_23		PORT_5	// P51
#define DIGITAL_PIN_24		PORT_5	// P50
#define DIGITAL_PIN_25		PORT_13	// P137
#define DIGITAL_PIN_26		PORT_14	// P140
#define DIGITAL_PIN_27		PORT_4	// P41
#define DIGITAL_PIN_28		PORT_7	// P73
#define DIGITAL_PIN_29		PORT_7	// P74
#define DIGITAL_PIN_30		PORT_7	// P75

/* Bit mask of digital pin define. */
#define DIGITAL_PIN_MASK_0	0x02	// P11
#define DIGITAL_PIN_MASK_1	0x04	// P12
#define DIGITAL_PIN_MASK_2	0x01	// P30
#define DIGITAL_PIN_MASK_3	0x40	// P16
#define DIGITAL_PIN_MASK_4	0x02	// P31
#define DIGITAL_PIN_MASK_5	0x20	// P15
#define DIGITAL_PIN_MASK_6	0x01	// P10
#ifndef GRCOTTON
#define DIGITAL_PIN_MASK_7	0x01	// P00
#define DIGITAL_PIN_MASK_8	0x02	// P01
#else
#define DIGITAL_PIN_MASK_7	0x04	// P02
#define DIGITAL_PIN_MASK_8	0x08	// P03
#endif // GRCOTTON
#define DIGITAL_PIN_MASK_9	0x08	// P13
#define DIGITAL_PIN_MASK_10	0x10	// P14
#define DIGITAL_PIN_MASK_11	0x04	// P72
#define DIGITAL_PIN_MASK_12	0x02	// P71
#define DIGITAL_PIN_MASK_13	0x01	// P70
#define DIGITAL_PIN_MASK_14	0x04	// P22
#define DIGITAL_PIN_MASK_15	0x08	// P23
#define DIGITAL_PIN_MASK_16	0x10	// P24
#define DIGITAL_PIN_MASK_17	0x20	// P25
#define DIGITAL_PIN_MASK_18	0x40	// P25
#define DIGITAL_PIN_MASK_19	0x80	// P27
#define DIGITAL_PIN_MASK_20	0x80	// P147
#define DIGITAL_PIN_MASK_21	0x01	// P120
#define DIGITAL_PIN_MASK_22	0x80	// P17
#define DIGITAL_PIN_MASK_23	0x02	// P51
#define DIGITAL_PIN_MASK_24	0x01	// P50
#define DIGITAL_PIN_MASK_25	0x80	// P137
#define DIGITAL_PIN_MASK_26	0x01	// P140
#define DIGITAL_PIN_MASK_27	0x02	// P41
#define DIGITAL_PIN_MASK_28	0x08	// P73
#define DIGITAL_PIN_MASK_29	0x10	// P74
#define DIGITAL_PIN_MASK_30	0x20	// P75
#endif

#define PWM_PIN_3			1		// TO1
#define PWM_PIN_5			2		// TO2
#define PWM_PIN_6			7		// TO7
#define PWM_PIN_9			4		// TO4
#ifdef GRADZUKI
#define PWM_PIN_10			0xE0	// Software PWM
#else
#define PWM_PIN_10			3		// TO3
#endif
#define PWM_PIN_11			0xE0	// Software PWM0
#define PWM_PIN_22			0xE1	// Software PWM1
#define PWM_PIN_23			0xE2	// Software PWM2
#define PWM_PIN_24			0xE3	// Software PWM3

#define SWPWM_PIN			0xE0


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
#ifndef __RL78__
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD, /* 0 */
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PB, /* 8 */
	PB,
	PB,
	PB,
	PB,
	PB,
	PC, /* 14 */
	PC,
	PC,
	PC,
	PC,
	PC,
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port D */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(0), /* 14, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER, /* 0 - port D */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	// on the ATmega168, digital pin 3 has hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
#else
	TIMER2B,
#endif
	NOT_ON_TIMER,
	// on the ATmega168, digital pins 5 and 6 have hardware pwm
#if defined(__AVR_ATmega8__)
	NOT_ON_TIMER,
	NOT_ON_TIMER,
#else
	TIMER0B,
	TIMER0A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 8 - port B */
	TIMER1A,
	TIMER1B,
#if defined(__AVR_ATmega8__)
	TIMER2,
#else
	TIMER2A,
#endif
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 14 - port C */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};
#else


const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
		DIGITAL_PIN_0,
		DIGITAL_PIN_1,
		DIGITAL_PIN_2,
		DIGITAL_PIN_3,
		DIGITAL_PIN_4,
		DIGITAL_PIN_5,
		DIGITAL_PIN_6,
		DIGITAL_PIN_7,
		DIGITAL_PIN_8,
		DIGITAL_PIN_9,
		DIGITAL_PIN_10,
		DIGITAL_PIN_11,
		DIGITAL_PIN_12,
		DIGITAL_PIN_13,
		DIGITAL_PIN_14,
		DIGITAL_PIN_15,
		DIGITAL_PIN_16,
		DIGITAL_PIN_17,
		DIGITAL_PIN_18,
		DIGITAL_PIN_19,
		DIGITAL_PIN_20,
		DIGITAL_PIN_21,
		DIGITAL_PIN_22,
		DIGITAL_PIN_23,
		DIGITAL_PIN_24,
		DIGITAL_PIN_25,
		DIGITAL_PIN_26,
		DIGITAL_PIN_27,
		DIGITAL_PIN_28,
		DIGITAL_PIN_29,
		DIGITAL_PIN_30
};
#undef _offset

#define _VB(mask) (uint8_t)(__builtin_log((uint8_t)mask) / __builtin_log(2))
const uint8_t PROGMEM digital_pin_to_bit_PGM[] = {
		_VB(DIGITAL_PIN_MASK_0),
		_VB(DIGITAL_PIN_MASK_1),
		_VB(DIGITAL_PIN_MASK_2),
		_VB(DIGITAL_PIN_MASK_3),
		_VB(DIGITAL_PIN_MASK_4),
		_VB(DIGITAL_PIN_MASK_5),
		_VB(DIGITAL_PIN_MASK_6),
		_VB(DIGITAL_PIN_MASK_7),
		_VB(DIGITAL_PIN_MASK_8),
		_VB(DIGITAL_PIN_MASK_9),
		_VB(DIGITAL_PIN_MASK_10),
		_VB(DIGITAL_PIN_MASK_11),
		_VB(DIGITAL_PIN_MASK_12),
		_VB(DIGITAL_PIN_MASK_13),
		_VB(DIGITAL_PIN_MASK_14),
		_VB(DIGITAL_PIN_MASK_15),
		_VB(DIGITAL_PIN_MASK_16),
		_VB(DIGITAL_PIN_MASK_17),
		_VB(DIGITAL_PIN_MASK_18),
		_VB(DIGITAL_PIN_MASK_19),
		_VB(DIGITAL_PIN_MASK_20),
		_VB(DIGITAL_PIN_MASK_21),
		_VB(DIGITAL_PIN_MASK_22),
		_VB(DIGITAL_PIN_MASK_23),
		_VB(DIGITAL_PIN_MASK_24),
		_VB(DIGITAL_PIN_MASK_25),
		_VB(DIGITAL_PIN_MASK_26),
		_VB(DIGITAL_PIN_MASK_27),
		_VB(DIGITAL_PIN_MASK_28),
		_VB(DIGITAL_PIN_MASK_29),
		_VB(DIGITAL_PIN_MASK_30)
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
		DIGITAL_PIN_MASK_0,
		DIGITAL_PIN_MASK_1,
		DIGITAL_PIN_MASK_2,
		DIGITAL_PIN_MASK_3,
		DIGITAL_PIN_MASK_4,
		DIGITAL_PIN_MASK_5,
		DIGITAL_PIN_MASK_6,
		DIGITAL_PIN_MASK_7,
		DIGITAL_PIN_MASK_8,
		DIGITAL_PIN_MASK_9,
		DIGITAL_PIN_MASK_10,
		DIGITAL_PIN_MASK_11,
		DIGITAL_PIN_MASK_12,
		DIGITAL_PIN_MASK_13,
		DIGITAL_PIN_MASK_14,
		DIGITAL_PIN_MASK_15,
		DIGITAL_PIN_MASK_16,
		DIGITAL_PIN_MASK_17,
		DIGITAL_PIN_MASK_18,
		DIGITAL_PIN_MASK_19,
		DIGITAL_PIN_MASK_20,
		DIGITAL_PIN_MASK_21,
		DIGITAL_PIN_MASK_22,
		DIGITAL_PIN_MASK_23,
		DIGITAL_PIN_MASK_24,
		DIGITAL_PIN_MASK_25,
		DIGITAL_PIN_MASK_26,
		DIGITAL_PIN_MASK_27,
		DIGITAL_PIN_MASK_28,
		DIGITAL_PIN_MASK_29,
		DIGITAL_PIN_MASK_30
};
#endif /*__RL78__*/


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
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif
#endif
