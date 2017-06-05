/*
  wiring_digital.c - digital input and output functions
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

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

  Modified 28 September 2010 by Mark Sproul

  $Id: wiring.c 248 2007-02-03 15:36:30Z mellis $
*/
/*
 * Modified  4 Mar  2017 by Yuuki Okamiya for RL78/G13
*/

#define ARDUINO_MAIN
#include "wiring_private.h"
#include "pins_arduino.h"

#ifdef __RL78__
#include "pintable.h"
extern bool g_u8AnalogWriteAvailableTable[NUM_DIGITAL_PINS];
extern volatile SwPwm g_SwPwm[NUM_SWPWM_PINS];

/**
 * PWMの停止
 *
 * @param[in] u8Timer 停止するPWMのタイマ番号
 *
 * @return なし
 *
 * @attention なし
 ***************************************************************************/
void _turnOffPwmPin(uint8_t u8Pin)
{
	unsigned int u16TMR0x;

	uint8_t u8Timer = 0xFF;
	if (u8Pin < NUM_DIGITAL_PINS) {
		u8Timer = getPinTable(u8Pin)->timer;
		g_u8AnalogWriteAvailableTable[u8Pin] = false;
	}
	if (u8Timer == SWPWM_PIN) {
		///////////////////////
		// Software PWM対応ピンの場合
		///////////////////////
#if defined(__RL78__)
		int i;

		for (i = 0; i < NUM_SWPWM_PINS; i++) {
			if (g_SwPwm[i].valid) {
				if (g_SwPwm[i].pin == u8Pin) {
					g_SwPwm[i].valid = false;
				} else {
					break;
				}
			}
		}
		if (i >= NUM_SWPWM_PINS) {			// SoftwarePWMの設定なし
			_stopTimerChannel(SW_PWM_TIMER);
		}
#endif
	} else {
		///////////////////////
		// PWM対応ピンの場合
		///////////////////////
		switch (u8Timer) {
		case 1:
			u16TMR0x = TMR01.tmr01;
			break;

		case 2:
			u16TMR0x = TMR02.tmr02;
			break;

		case 3:
			u16TMR0x = TMR03.tmr03;
			break;

		case 4:
			u16TMR0x = TMR04.tmr04;
			break;

		case 5:
			u16TMR0x = TMR05.tmr05;
			break;

		case 6:
			u16TMR0x = TMR06.tmr06;
			break;

		case 7:
			u16TMR0x = TMR07.tmr07;
			break;

		default:
			u16TMR0x = PWM_MASTER_MODE;
			break;
		}
		if (u16TMR0x == PWM_SLAVE_MODE) {
			_stopTimerChannel(u8Timer);
		}
	}
}
#endif/*__RL78__*/

void pinMode(uint8_t pin, uint8_t u8Mode)
{
#ifndef __RL78__
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return;

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mode == INPUT) { 
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	} else if (mode == INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                cli();
		*reg |= bit;
		SREG = oldSREG;
	}
#else /*__RL78__*/
	if (pin < NUM_DIGITAL_PINS) {
		// アナログピンかどうか？
		if (14 <= pin && pin <= 21) {
			// ピンモードをデジタルモードに変更
			if (pin == 20) {
				PMC14.pmc14 &= ~0x80;// P147をデジタルポートに設定
			}
			else if (pin == 21) {
				PMC12.pmc12 &= ~0x01;// P120をデジタルポートに設定
			}
			else {
				uint8_t oldadpc = ADPC.adpc;
				uint8_t newadpc = (pin - 14) + ANALOG_ADPC_OFFSET - 1;
				if ((oldadpc  == 0x00 ) || (oldadpc > newadpc)) {
					ADPC.adpc = newadpc;
				}
			}
		}

		PinTableType* p = getPinTable(pin);
		{
			if (g_u8AnalogWriteAvailableTable[pin]) {
				_turnOffPwmPin(pin);	// PWMの設定解除
			}
#ifdef WORKAROUND_READ_MODIFY_WRITE
#if 0
			if (u8Mode == INPUT) {
				sbi(p->portModeRegisterAddr,  p->mask);	// 入力モードに設定
				sbi(p->portInputModeRegisterAddr, p->mask);	// TTL入力バッファに設定
				cbi(p->portPullUpRegisterAddr,  p->mask);	// プルアップ抵抗を無効に設定
			} else if (u8Mode == INPUT_PULLUP) {
				sbi(p->portModeRegisterAddr,  p->mask);	// 入力モードに設定
				cbi(p->portInputModeRegisterAddr, p->mask);	// CMOS入力バッファに設定
				sbi(p->portPullUpRegisterAddr,  p->mask);	// プルアップ抵抗を有効に設定
			} else {
				cbi(p->portModeRegisterAddr,  p->mask);	// 出力モードに設定
				cbi(p->portOutputModeRegisterAddr, p->mask);	// 通常出力モードに設定
			}
#else
			switch (u8Mode) {
			case INPUT:
			case INPUT_PULLUP:
			case INPUT_TTL:
			case INPUT_TTL_PULLUP:
				sbi(p->portModeRegisterAddr,  p->mask);	// 入力モードに設定
				if (u8Mode == INPUT_PULLUP || u8Mode == INPUT_TTL_PULLUP) {
					sbi(p->portPullUpRegisterAddr,  p->mask);	// プルアップ抵抗を有効に設定
				} else {
					cbi(p->portPullUpRegisterAddr,  p->mask);	// プルアップ抵抗を無効に設定
				}
				if (u8Mode == INPUT_TTL || u8Mode == INPUT_TTL_PULLUP) {
					sbi(p->portInputModeRegisterAddr, p->mask);	// TTL入力バッファに設定
				} else {
					cbi(p->portInputModeRegisterAddr, p->mask);	// CMOS入力バッファに設定
				}
				break;
			case OUTPUT:
			case OUTPUT_OPENDRAIN:
				cbi(p->portModeRegisterAddr,  p->mask);	// 出力モードに設定
				if (u8Mode == OUTPUT_OPENDRAIN) {
					sbi(p->portOutputModeRegisterAddr, p->mask);	// N-chオープン・ドレイン出力モードに設定
				} else {
					cbi(p->portOutputModeRegisterAddr, p->mask);	// 通常出力モードに設定
				}
				break;
			}
#endif
//			cbi(p->portRegisterAddr, p->mask);			// 出力をLOWに設定
#else
			if (u8Mode == INPUT) {
				*p->portModeRegisterAddr  |= p->mask;		// 入力モードに設定
				*p->portInputModeRegisterAddr |= p->mask;		// TTL入力バッファに設定
				*p->portPullUpRegisterAddr  &= ~p->mask;	// プルアップ抵抗を無効に設定
			} else if (u8Mode == INPUT_PULLUP) {
				*p->portModeRegisterAddr  |= p->mask;		// 入力モードに設定
				*p->portInputModeRegisterAddr &= ~p->mask;	// CMOS入力バッファに設定
				*p->portPullUpRegisterAddr  |= p->mask;		// プルアップ抵抗を有効に設定
			} else {
				*p->portModeRegisterAddr  &= ~p->mask;	// 出力モードに設定
				*p->portOutputModeRegisterAddr &=  ~p->mask;	// 通常出力モードに設定
			}
//			*p->portRegisterAddr &= ~p->mask;			// 出力をLOWに設定
#endif
		}
	}
#endif/*__RL78__*/
}

// Forcing this inline keeps the callers from having to push their own stuff
// on the stack. It is a good performance win and only takes 1 more byte per
// user than calling. (It will take more bytes on the 168.)
//
// But shouldn't this be moved into pinMode? Seems silly to check and do on
// each digitalread or write.
//
// Mark Sproul:
// - Removed inline. Save 170 bytes on atmega1280
// - changed to a switch statment; added 32 bytes but much easier to read and maintain.
// - Added more #ifdefs, now compiles for atmega645
//
//static inline void turnOffPWM(uint8_t timer) __attribute__ ((always_inline));
//static inline void turnOffPWM(uint8_t timer)
static void turnOffPWM(uint8_t pin)
{
#ifndef __RL78__
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TIMER0B) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
#else /*__RL78__*/
	unsigned int u16TMR0x;

	uint8_t u8Timer = 0xFF;
	if (pin < NUM_DIGITAL_PINS) {
		u8Timer = getPinTable(pin)->timer;
		g_u8AnalogWriteAvailableTable[pin] = false;
	}
	if (u8Timer == SWPWM_PIN) {
		///////////////////////
		// Software PWM対応ピンの場合
		///////////////////////
#if defined(__RL78__)
		int i;

		for (i = 0; i < NUM_SWPWM_PINS; i++) {
			if (g_SwPwm[i].valid) {
				if (g_SwPwm[i].pin == pin) {
					g_SwPwm[i].valid = false;
				} else {
					break;
				}
			}
		}
		if (i >= NUM_SWPWM_PINS) {			// SoftwarePWMの設定なし
			_stopTimerChannel(SW_PWM_TIMER);
		}
#endif
	} else {
		///////////////////////
		// PWM対応ピンの場合
		///////////////////////
		switch (u8Timer) {
		case 1:
			u16TMR0x = TMR01.tmr01;
			break;

		case 2:
			u16TMR0x = TMR02.tmr02;
			break;

		case 3:
			u16TMR0x = TMR03.tmr03;
			break;

		case 4:
			u16TMR0x = TMR04.tmr04;
			break;

		case 5:
			u16TMR0x = TMR05.tmr05;
			break;

		case 6:
			u16TMR0x = TMR06.tmr06;
			break;

		case 7:
			u16TMR0x = TMR07.tmr07;
			break;

		default:
			u16TMR0x = PWM_MASTER_MODE;
			break;
		}
		if (u16TMR0x == PWM_SLAVE_MODE) {
			_stopTimerChannel(u8Timer);
		}
	}
#endif/*__RL78__*/
}

void digitalWrite(uint8_t pin, uint8_t val)
{
#ifndef __RL78__
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	cli();

	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	SREG = oldSREG;
#else /*__RL78__*/
#if 0
	if (pin < NUM_DIGITAL_PINS) {
		PinTableType* p = getPinTable(pin);
		{
			if (*p->portModeRegisterAddr & p->mask) {
				// 入力モードの場合
#ifdef WORKAROUND_READ_MODIFY_WRITE
				if (val == LOW) {
					cbi(p->portPullUpRegisterAddr, p->mask);	// プルアップ抵抗を無効に設定
					cbi(p->portRegisterAddr, p->mask);		// 出力をLOWに設定
				} else {
					sbi(p->portPullUpRegisterAddr, p->mask);	// プルアップ抵抗を有効に設定
					sbi(p->portRegisterAddr, p->mask);		// 出力をHIGHに設定
				}
#else
				if (val == LOW) {
					*p->portPullUpRegisterAddr &= ~p->mask;		// プルアップ抵抗を無効に設定
				} else {
					*p->portPullUpRegisterAddr |= p->mask;		// プルアップ抵抗を有効に設定
				}
#endif
			}
			else {
				// 出力モードの場合
#ifdef WORKAROUND_READ_MODIFY_WRITE
				if (val == LOW) {
					cbi(p->portRegisterAddr, p->mask);		// 出力をLOWに設定
				} else {
					sbi(p->portRegisterAddr, p->mask);		// 出力をHIGHに設定
				}
#else
				if (val == LOW) {
					*p->portRegisterAddr &= ~p->mask;		// 出力をLOWに設定
				} else {
					*p->portRegisterAddr |= p->mask;		// 出力をHIGHに設定
				}
#endif
			}
		}
	}
#else
	__asm __volatile(
	    "\n"
	    "        /* %%0 = %0 */                  \n"
	    "        /* %%1 = %1 */                  \n"
	    "        /* %%2 = %2 */                  \n"
	    "        /* %%3 = %3 */                  \n"
	    "        /* %%4 = %4 */                  \n"
	    "        /* %%5 = %5 */                  \n"
	    "        /* %%6 = %6 */                  \n"
	    "        /* %%7 = %7 */                  \n"
	    "        /* %%8 = %8 */                  \n"
	    "        mov     a, %0                   \n"
	    "        cmp     a, %1                   \n"
	    "        bnc     $8f                     \n"
	    "        mov     x, %2                   \n"
	    "        mulu    x                       \n"
	    "        movw    bc, ax                  \n"
	    "        movw    ax, %u3[bc]             \n"
	    "        movw    hl, ax                  \n"
	    "        mov     a, %u7[bc]              \n"
	    "        and     a, [hl]                 \n"
	    "        movw    ax, %u5[bc]             \n"
	    "        skz                             \n"
	    "        movw    ax, %u4[bc]             \n"
	    "        movw    hl, ax                  \n"
	    "        mov     a, %8                   \n"
	    "        add     a, #0xff                \n"
	    "        mov     a, %u6[bc]              \n"
	    "        bt      a.2, $4f                \n"
	    "        bt      a.1, $2f                \n"
	    "        bt      a.0, $1f                \n"
	    "        mov1    [hl].0, cy              \n"
	    "        br      $8f                     \n"
	    "1:                                      \n"
	    "        mov1    [hl].1, cy              \n"
	    "        br      $8f                     \n"
	    "2:                                      \n"
	    "        bt      a.0, $3f                \n"
	    "        mov1    [hl].2, cy              \n"
	    "        br      $8f                     \n"
	    "3:                                      \n"
	    "        mov1    [hl].3, cy              \n"
	    "        br      $8f                     \n"
	    "4:                                      \n"
	    "        bt      a.1, $6f                \n"
	    "        bt      a.0, $5f                \n"
	    "        mov1    [hl].4, cy              \n"
	    "        br      $8f                     \n"
	    "5:                                      \n"
	    "        mov1    [hl].5, cy              \n"
	    "        br      $8f                     \n"
	    "6:                                      \n"
	    "        bt      a.0, $7f                \n"
	    "        mov1    [hl].6, cy              \n"
	    "        br      $8f                     \n"
	    "7:                                      \n"
	    "        mov1    [hl].7, cy              \n"
	    "8:                                      \n"
	    :
	    : "m"(pin),
	      "i"(NUM_DIGITAL_PINS),
	      "i"((unsigned)&PinTable[1] - (unsigned)&PinTable[0]),
	      "i"(&PinTable->portModeRegisterAddr),
	      "i"(&PinTable->portPullUpRegisterAddr),
	      "i"(&PinTable->portRegisterAddr),
	      "i"(&PinTable->bit),
	      "i"(&PinTable->mask),
	      "m"(val)
	    : "a", "x", "b", "c", "h", "l"
	);
#endif
#endif /*__RL78__*/
}

int digitalRead(uint8_t pin)
{
#ifndef __RL78__
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
#else
#if 0
	int	s16Value;
	if (pin < NUM_DIGITAL_PINS) {
		PinTableType* p = getPinTable(pin);
		if (*p->portRegisterAddr & p->mask) {
#if 1
			__asm __volatile(
				"movw %0, %1 \n"
				: "=r"(s16Value)
				: "i"(HIGH)
			);
			return s16Value;
#else
			return HIGH;
#endif
		}
	}

#if 1
	__asm __volatile(
		"movw %0, %1 \n"
		: "=r"(s16Value)
		: "i"(LOW)
	);
	return s16Value;
#else
	return LOW;
#endif
#else
	int value = 0;
	__asm __volatile(
	    "\n"
	    "        /* %%0 = %0 */                 \n"
	    "        /* %%1 = %1 */                 \n"
	    "        /* %%2 = %2 */                 \n"
	    "        /* %%3 = %3 */                 \n"
	    "        /* %%4 = %4 */                 \n"
	    "        /* %%5 = %5 */                 \n"
	    "        /* %%6 = %6 */                 \n"
	    "        mov     a, %2                  \n"
	    "        cmp     a, %3                  \n"
	    "        bnc     $9f                    \n"
	    "        mov     x, %4                  \n"
	    "        mulu    x                      \n"
	    "        movw    bc, ax                 \n"
	    "        movw    ax, %u5[bc]            \n"
	    "        movw    hl, ax                 \n"
	    "        mov     a, %u6[bc]             \n"
	    "        and     a, [hl]                \n"
	    "        add     a, #0xff               \n"
	    "        mov1    %1.0, cy               \n"
	    "9:                                     \n"
	    : "=r"(value)
	    : "r"(value),
	      "m"(pin),
	      "i"(NUM_DIGITAL_PINS),
	      "i"((unsigned)&PinTable[1] - (unsigned)&PinTable[0]),
	      "i"(&PinTable->portRegisterAddr),
	      "i"(&PinTable->mask)
	    : "a", "x", "b", "c", "h", "l"
	);
	return value;
#endif

#endif /*__RL78__*/
}
