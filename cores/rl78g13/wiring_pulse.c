/*
  wiring_pulse.c - pulseIn() function
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

  $Id: wiring.c 248 2007-02-03 15:36:30Z mellis $
*/
/*
 * Modified  5 Mar  2017 by Yuuki Okamiya for RL78/G13
　* Modified 18 Mar  2017 by Nozomu Fujita for pulseInWait, pulseInCount
*/

#include "wiring_private.h"
#include "pins_arduino.h"
#include "pintable.h"

#ifdef __RL78__
static const uint16_t PulseInWaitCycles = 1 + 1 + 1 + 2 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 4;

static bool pulseInWait(
	volatile uint8_t* Px,
	uint8_t u8Bit,
	uint8_t u8StateMask,
	unsigned long* u32TimeoutCycles
) {
	bool ret;
#if 0
	const unsigned char u16PulseInWaitCycles = 30;
	while ((*Px & u8Bit) == u8StateMask) {
		if (*u32TimeoutCycles >= u16PulseInWaitCycles) {
			*u32TimeoutCycles -= u16PulseInWaitCycles;
		} else {
			ret = true;
			break;
		}
	}
	ret = false;
#else
	__asm __volatile(
		"	clrb	%0					\n"
		"	movw	de, %1					\n"
		"	mov	a, %2					\n"
		"	mov	b, a					\n"
		"	mov	a, %3					\n"
		"	mov	c, a					\n"
		"	movw	hl, %4					\n"
		"1:							\n"
		"	mov	a, [de]				;1	\n"
		"	and	a, b				;1	\n"
		"	cmp	a, c				;1	\n"
		"	bnz	$2f				;2/4	\n"
		"	movw	ax, [hl]			;1	\n"
		"	subw	ax, %5				;1	\n"
		"	movw	[hl], ax			;1	\n"
		"	movw	ax, [hl+2]			;1	\n"
		"	sknc					;1	\n"
		"	subw	ax, #1				;1	\n"
		"	movw	[hl+2], ax			;1	\n"
		"	bnc	$1b				;2/4	\n"
		"	oneb	%0					\n"
		"2:		 					\n"
		:"=&r"(ret)
		:"r"(Px), "r"(u8Bit), "r"(u8StateMask), "r"(u32TimeoutCycles), "i"(PulseInWaitCycles)
		:"ax", "bc", "de", "hl"
	);
#endif
	return ret;
}

static const uint16_t PulseInCountCycles = 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 2 + 1 + 1 + 1 + 1 + 1 + 1 + 1 + 4;

static bool pulseInCount(
	volatile uint8_t* Px,
	uint8_t u8Bit,
	uint8_t u8StateMask,
	unsigned long u32TimeoutCycles,
	unsigned long* u32PulseCycles
) {
	bool ret;
#if 0
	const unsigned long u16PulseInCountCycles = 35;
	while ((*Px & u8Bit) == u8StateMask) {
		if (u32TimeoutCycles >= u16PulseInCountCycles) {
			u32TimeoutCycles -= u16PulseInCountCycles;
		} else {
			ret = true;
			break;
		}
		*u32PulseCycles += u16PulseInCountCycles;
	}
	ret = false;
#else
	__asm __volatile(
		"	clrb	%0					\n"
		"	movw	de, %1					\n"
		"	mov	a, %2					\n"
		"	mov	b, a					\n"
		"	mov	a, %3					\n"
		"	mov	c, a					\n"
		"	movw	hl, %5					\n"
		"	br	$2f					\n"
		"1:							\n"
		"	movw	ax, [hl]			;1	\n"
		"	addw	ax, %7				;1	\n"
		"	movw	[hl], ax			;1	\n"
		"	movw	ax, [hl+2]			;1	\n"
		"	sknc					;1	\n"
		"	incw	ax				;1	\n"
		"	movw	[hl+2], ax			;1	\n"
		"2:							\n"
		"	mov	a, [de]				;1	\n"
		"	and	a, b				;1	\n"
		"	cmp	a, c				;1	\n"
		"	bnz	$3f				;2/4	\n"
		"	movw	ax, %4+0			;1	\n"
		"	subw	ax, %7				;1	\n"
		"	movw	%4+0, ax			;1	\n"
		"	movw	ax, %4+2			;1	\n"
		"	sknc					;1	\n"
		"	subw	ax, #1				;1	\n"
		"	movw	%4+2, ax			;1	\n"
		"	bnc	$1b				;2/4	\n"
		"	oneb	%0					\n"
		"3:		 					\n"
		:"=&r"(ret)
		:"r"(Px), "r"(u8Bit), "r"(u8StateMask), "r"(u32TimeoutCycles), "r"(u32PulseCycles), "i"(PulseInWaitCycles), "i"(PulseInCountCycles)
		:"ax", "bc", "de", "hl"
	);
#endif
	return ret;
}

#endif
/* Measures the length (in microseconds) of a pulse on the pin; state is HIGH
 * or LOW, the type of pulse to measure.  Works on pulses from 2-3 microseconds
 * to 3 minutes in length, but must be called at least a few dozen microseconds
 * before the start of the pulse. */
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
#ifndef __RL78__
	// cache the port and bit of the pin in order to speed up the
	// pulse width measuring loop and achieve finer resolution.  calling
	// digitalRead() instead yields much coarser resolution.
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	uint8_t stateMask = (state ? bit : 0);
	unsigned long width = 0; // keep initialization out of time critical area
	
	// convert the timeout from microseconds to a number of times through
	// the initial loop; it takes 16 clock cycles per iteration.
	unsigned long numloops = 0;
	unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;
	
	// wait for any previous pulse to end
	while ((*portInputRegister(port) & bit) == stateMask)
		if (numloops++ == maxloops)
			return 0;
	
	// wait for the pulse to start
	while ((*portInputRegister(port) & bit) != stateMask)
		if (numloops++ == maxloops)
			return 0;
	
	// wait for the pulse to stop
	while ((*portInputRegister(port) & bit) == stateMask) {
		if (numloops++ == maxloops)
			return 0;
		width++;
	}

	// convert the reading to microseconds. The loop has been determined
	// to be 20 clock cycles long and have about 16 clocks between the edge
	// and the start of the loop. There will be some error introduced by
	// the interrupt handlers.
	return clockCyclesToMicroseconds(width * 21 + 16); 
#else
	/**
	 * ピンに入力されるパルスを検出します。
	 *
	 * たとえば、パルスの種類(u8Value)をHIGHに指定した場合、 pulseIn()関数は入力がHIGHに
	 * 変わると同時に時間の計測を始め、またLOWに戻ったら、そこまでの時間(つまりパルス
	 * の長さ)をマイクロ秒単位で返します。タイムアウトを指定した場合は、その時間を超
	 * えた時点で0を返します。
	 *
	 * @param[in] u8Pin      パルスを入力するピン番号を指定します。
	 * @param[in] u8Value    測定するパルスの種類（HIGHまたはLOW）を指定します。
	 * @param[in] u32Timeout タイムアウトまでの時間(単位・マイクロ秒)を指定します。
	 *
	 * @return パルスの長さ(マイクロ秒)を返却します。
	 *         パルスがスタートする前にタイムアウトとなった場合は0を返却します。
	 *
	 * @attention なし
	 ***************************************************************************/
	uint8_t u8StateMask;
	unsigned long u32TimeoutCycles;
	unsigned long u32PulseCycles = 0;
	unsigned long u32PulseLength = 0;
	const unsigned long u32AdjustCycles = 50 + PulseInWaitCycles / 2 + PulseInCountCycles / 2;
	bool timeOut;

	if (pin < NUM_DIGITAL_PINS) {
		PinTableType* p = getPinTable(pin);
		u8StateMask = (state ? p->mask : 0);
		{
			u32TimeoutCycles = microsecondsToClockCycles(timeout);
			timeOut =  pulseInWait(p->portRegisterAddr, p->mask, u8StateMask, &u32TimeoutCycles);

			if (!timeOut) {
				timeOut =  pulseInWait(p->portRegisterAddr, p->mask, u8StateMask ^ p->mask, &u32TimeoutCycles);
				if (!timeOut) {
					timeOut = pulseInCount(p->portRegisterAddr, p->mask, u8StateMask, u32TimeoutCycles, &u32PulseCycles);
				}
			}
			if (!timeOut) {
				u32PulseLength = clockCyclesToMicroseconds(u32PulseCycles + u32AdjustCycles);
			}
		}
	}

	return u32PulseLength;
#endif
}
