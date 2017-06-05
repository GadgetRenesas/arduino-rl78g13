/*
 wiring_analog.c - analog input and output
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

#include "wiring_private.h"
#include "pins_arduino.h"

#ifdef __RL78__
#include "pintable.h"
static uint8_t g_u8AnalogReference = DEFAULT;
static uint8_t g_u8SwPwmTicksCount = 0;
uint8_t g_u8ADUL = 0xFF;
uint8_t g_u8ADLL = 0x00;
boolean g_bAdcInterruptFlag = false;
static int _analogRead(uint8_t u8ADS);
#endif /*__RL78__*/

void analogReference(uint8_t mode) {
	// can't actually set the register here because the default setting
	// will connect AVCC and the AREF pin, which would cause a short if
	// there's something connected to AREF.
#ifndef __RL78__
	analog_reference = mode;
#else
	g_u8AnalogReference = mode;
#endif
}

#ifdef __RL78__

bool g_u8AnalogWriteAvailableTable[NUM_DIGITAL_PINS] = {
false, false, false, false,
false, false, false, false,
false, false, false, false,
false, false, false, false,
false, false, false, false,
false, false, false, false,
false, };
const uint8_t g_au8AnalogPinTable[NUM_ANALOG_INPUTS] = {
ANALOG_PIN_0, ANALOG_PIN_1, ANALOG_PIN_2, ANALOG_PIN_3,
ANALOG_PIN_4, ANALOG_PIN_5,
ANALOG_PIN_6, ANALOG_PIN_7, };

volatile SwPwm g_SwPwm[NUM_SWPWM_PINS] = { { 0, 0, 0, 0, 0, 0 }, };

bool g_u8AnalogReadAvailableTable[NUM_ANALOG_INPUTS] = { 0 };

// PWMの周期(TDR00の設定値)
static uint16_t g_u16TDR00 = PWM_TDR00;

#endif/*__RL78__*/

int analogRead(uint8_t pin) {
#ifndef __RL78__
	uint8_t low, high;

#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
	if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#endif
	pin = analogPinToChannel(pin);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
	if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
	if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#else
	if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif

#if defined(ADCSRB) && defined(MUX5)
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif

	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
#if defined(ADMUX)
	ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif

	// without a delay, we seem to read from the wrong channel
	//delay(1);

#if defined(ADCSRA) && defined(ADCL)
	// start the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low = ADCL;
	high = ADCH;
#else
	// we dont have an ADC, return 0
	low = 0;
	high = 0;
#endif

	// combine the two bytes
	return (high << 8) | low;
#else /*__RL78__*/
	int s16Result = 0;
	uint8_t oldadpc, newadpc;
	// ピン番号がANALOG_PIN_START_NUMBER（GR-KURUMIの場合は14）以上の場合0～に割り当てなおす。
	if (pin >= ANALOG_PIN_START_NUMBER) {
		pin -= ANALOG_PIN_START_NUMBER;
	}

	if (pin < NUM_ANALOG_INPUTS) {
		if (g_u8AnalogReadAvailableTable[pin] == false) {
			// ピンモードをAnalogモードに変更
			switch (g_au8AnalogPinTable[pin]) {
			case 18:
				PMC14.pmc14 |= 0x80;	// P147をアナログポートに設定
				break;

			case 19:
				PMC12.pmc12 |= 0x01;	// P120をアナログポートに設定
				break;

			default:
				oldadpc = ADPC.adpc;
				newadpc = (pin - 14) + ANALOG_ADPC_OFFSET - 1;
				if (newadpc > oldadpc) {
					ADPC.adpc = newadpc;
				}
				break;
			}
			// Specify pin number as digital port number
			pinMode(pin + ANALOG_PIN_START_NUMBER, INPUT);
			g_u8AnalogReadAvailableTable[pin] = true;

		}

	}

	// アナログ値の読み込み
	s16Result = _analogRead(g_au8AnalogPinTable[pin]);
	return s16Result;
#endif/*__RL78__*/
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint8_t pin, int val) {
#ifndef __RL78__
	// We need to make sure the PWM output is enabled for those pins
	// that support it, as we turn it off when digitally reading or
	// writing with them.  Also, make sure the pin is in output mode
	// for consistenty with Wiring, which doesn't require a pinMode
	// call for the analog output pins.
	pinMode(pin, OUTPUT);
	if (val == 0)
	{
		digitalWrite(pin, LOW);
	}
	else if (val == 255)
	{
		digitalWrite(pin, HIGH);
	}
	else
	{
		switch(digitalPinToTimer(pin))
		{
			// XXX fix needed for atmega8
#if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
			case TIMER0A:
			// connect pwm to pin on timer 0
			sbi(TCCR0, COM00);
			OCR0 = val;// set pwm duty
			break;
#endif

#if defined(TCCR0A) && defined(COM0A1)
			case TIMER0A:
			// connect pwm to pin on timer 0, channel A
			sbi(TCCR0A, COM0A1);
			OCR0A = val;// set pwm duty
			break;
#endif

#if defined(TCCR0A) && defined(COM0B1)
			case TIMER0B:
			// connect pwm to pin on timer 0, channel B
			sbi(TCCR0A, COM0B1);
			OCR0B = val;// set pwm duty
			break;
#endif

#if defined(TCCR1A) && defined(COM1A1)
			case TIMER1A:
			// connect pwm to pin on timer 1, channel A
			sbi(TCCR1A, COM1A1);
			OCR1A = val;// set pwm duty
			break;
#endif

#if defined(TCCR1A) && defined(COM1B1)
			case TIMER1B:
			// connect pwm to pin on timer 1, channel B
			sbi(TCCR1A, COM1B1);
			OCR1B = val;// set pwm duty
			break;
#endif

#if defined(TCCR2) && defined(COM21)
			case TIMER2:
			// connect pwm to pin on timer 2
			sbi(TCCR2, COM21);
			OCR2 = val;// set pwm duty
			break;
#endif

#if defined(TCCR2A) && defined(COM2A1)
			case TIMER2A:
			// connect pwm to pin on timer 2, channel A
			sbi(TCCR2A, COM2A1);
			OCR2A = val;// set pwm duty
			break;
#endif

#if defined(TCCR2A) && defined(COM2B1)
			case TIMER2B:
			// connect pwm to pin on timer 2, channel B
			sbi(TCCR2A, COM2B1);
			OCR2B = val;// set pwm duty
			break;
#endif

#if defined(TCCR3A) && defined(COM3A1)
			case TIMER3A:
			// connect pwm to pin on timer 3, channel A
			sbi(TCCR3A, COM3A1);
			OCR3A = val;// set pwm duty
			break;
#endif

#if defined(TCCR3A) && defined(COM3B1)
			case TIMER3B:
			// connect pwm to pin on timer 3, channel B
			sbi(TCCR3A, COM3B1);
			OCR3B = val;// set pwm duty
			break;
#endif

#if defined(TCCR3A) && defined(COM3C1)
			case TIMER3C:
			// connect pwm to pin on timer 3, channel C
			sbi(TCCR3A, COM3C1);
			OCR3C = val;// set pwm duty
			break;
#endif

#if defined(TCCR4A)
			case TIMER4A:
			//connect pwm to pin on timer 4, channel A
			sbi(TCCR4A, COM4A1);
#if defined(COM4A0)		// only used on 32U4
			cbi(TCCR4A, COM4A0);
#endif
			OCR4A = val;	// set pwm duty
			break;
#endif

#if defined(TCCR4A) && defined(COM4B1)
			case TIMER4B:
			// connect pwm to pin on timer 4, channel B
			sbi(TCCR4A, COM4B1);
			OCR4B = val;// set pwm duty
			break;
#endif

#if defined(TCCR4A) && defined(COM4C1)
			case TIMER4C:
			// connect pwm to pin on timer 4, channel C
			sbi(TCCR4A, COM4C1);
			OCR4C = val;// set pwm duty
			break;
#endif

#if defined(TCCR4C) && defined(COM4D1)
			case TIMER4D:
			// connect pwm to pin on timer 4, channel D
			sbi(TCCR4C, COM4D1);
#if defined(COM4D0)		// only used on 32U4
			cbi(TCCR4C, COM4D0);
#endif
			OCR4D = val;	// set pwm duty
			break;
#endif

#if defined(TCCR5A) && defined(COM5A1)
			case TIMER5A:
			// connect pwm to pin on timer 5, channel A
			sbi(TCCR5A, COM5A1);
			OCR5A = val;// set pwm duty
			break;
#endif

#if defined(TCCR5A) && defined(COM5B1)
			case TIMER5B:
			// connect pwm to pin on timer 5, channel B
			sbi(TCCR5A, COM5B1);
			OCR5B = val;// set pwm duty
			break;
#endif

#if defined(TCCR5A) && defined(COM5C1)
			case TIMER5C:
			// connect pwm to pin on timer 5, channel C
			sbi(TCCR5A, COM5C1);
			OCR5C = val;// set pwm duty
			break;
#endif

			case NOT_ON_TIMER:
			default:
			if (val < 128) {
				digitalWrite(pin, LOW);
			} else {
				digitalWrite(pin, HIGH);
			}
		}
	}
#else /*__RL78__*/
	uint8_t u8Timer;
	unsigned short u16Duty;

	if (pin < NUM_DIGITAL_PINS) {
		{
			val = min(max(val, PWM_MIN), PWM_MAX);
			u8Timer = getPinTable(pin)->timer;
			if (u8Timer == SWPWM_PIN) {
				///////////////////////
				// Software PWM対応ピンの場合
				///////////////////////
#if defined(__RL78__)
				_startTAU0(TIMER_CLOCK);
				if (!g_u8AnalogWriteAvailableTable[pin]) {
					pinMode(pin, OUTPUT);		// 初期時のみ出力モードを設定
					g_u8AnalogWriteAvailableTable[pin] = true;
				}
				int i;
				int j = NUM_SWPWM_PINS;
				for (i = 0; i < NUM_SWPWM_PINS; i++) {
					if (!g_SwPwm[i].valid) {
						if (j >= NUM_SWPWM_PINS) {
							j = i;
						}
					} else if (g_SwPwm[i].pin == pin) {
						break;
					}
				}
				if (i >= NUM_SWPWM_PINS) {
					i = j;
				}
				if (i < NUM_SWPWM_PINS) {
					if (g_SwPwm[i].valid && g_SwPwm[i].pin == pin) {
						g_SwPwm[i].newValue = val;
					} else {
						g_SwPwm[i].valid = false;
						g_SwPwm[i].pin = pin;
						g_SwPwm[i].value = g_SwPwm[i].newValue = val;
						g_SwPwm[i].port = getPinTable(pin)->portRegisterAddr;
						g_SwPwm[i].mask = getPinTable(pin)->mask;
						g_SwPwm[i].valid = true;
					}
					if (!(TE0.te0 & 0x0040)) { // No pin uses Software PWM
						_startTimerChannel( SW_PWM_TIMER, 0x0001, SWPWM_MIN,
								false, true);
					}
				} else {
					digitalWrite(pin,
							val >= ((PWM_MIN + PWM_MAX + 1) / 2) ? HIGH : LOW);
				}

#endif
			} else {
				///////////////////////
				// PWM対応ピンの場合
				///////////////////////
				_startTAU0(TIMER_CLOCK);

				if (((TE0.te0 & 0x0001) == 0) || (TDR00.tdr00 != g_u16TDR00)) {
					// Masterチャネルの設定
					TT0.tt0 |= 0x0001;			// タイマ停止
					TMR00.tmr00 = PWM_MASTER_MODE;	// 動作モードの設定
					TDR00.tdr00 = g_u16TDR00;		// PWM出力の周期の設定
					TO0.to0 &= ~0x0001;			// タイマ出力の設定
					TOE0.toe0 &= ~0x0001;			// タイマ出力許可の設定
					// マスタチャネルのタイマ動作許可
					TS0.ts0 |= 0x00001;
				}

				u16Duty = (unsigned short) (((unsigned long) val
						* (g_u16TDR00 + 1)) / PWM_MAX);
				if (!g_u8AnalogWriteAvailableTable[pin]) {
					pinMode(pin, OUTPUT);			// 出力モードに設定
					digitalWrite(pin, LOW);

					// Slaveチャネルの設定
					_startTimerChannel(u8Timer, PWM_SLAVE_MODE, u16Duty, true,
							false);
					g_u8AnalogWriteAvailableTable[pin] = true;

				} else {
					_modifyTimerPeriodic(u8Timer, u16Duty);
				}
			}
		}
	}

#endif/*__RL78__*/
}

#ifdef __RL78__
void analogWriteFrequency(uint32_t u32Hz) {
	// PWM出力パルス周期設定
	//   パルス周期 = (TDR00の設定値+1) x カウント・クロック周期
	//   例）パルス周期が2[ms]の場合
	//       2[ms] = 1/32[MHz] x (TDR00の設定値 + 1)
	//       TDR00の設定値 = 63999
	if (u32Hz > PWM_MASTER_CLOCK) {
		g_u16TDR00 = 0x0000;
	} else {
		g_u16TDR00 = (PWM_MASTER_CLOCK / u32Hz) - 1;
	}
}

static int _analogRead(uint8_t u8ADS) {
	int s16Result = 0;

	if (((0 <= u8ADS) && (u8ADS <= 14)) || ((16 <= u8ADS) && (u8ADS <= 26))) {
#ifdef WORKAROUND_READ_MODIFY_WRITE
		ADM0.adm0 = 0x00;		// A/Dコンバータの動作停止、fclk/64、ノーマル1モードに設定
		CBI(SFR_IF1H, 0);		// INTADの割り込みフラグのクリア
#else /* WORKAROUND_READ_MODIFY_WRITE */
		ADCEN = 1;			// A/Dコンバータにクロック供給開始
		NOP();
		NOP();
		NOP();
		NOP();
		ADM0.adm0 = 0x00;// A/Dコンバータの動作停止、fclk/64、ノーマル1モードに設定
		ADMK = 1;// INTADの割り込み禁止
		ADIF = 0;// INTADの割り込みフラグのクリア
		ADPR1 = 1;// INTADの割り込み優先順位の設定
		ADPR0 = 1;// INTADの割り込み優先順位の設定
#endif
		if (g_u8AnalogReference == DEFAULT) {
			ADM2.adm2 = 0x00;	// Vddリファレンスに設定
		} else if (g_u8AnalogReference == INTERNAL) {
			ADM2.adm2 = 0x80;	// 内部リファレンス(1.45V)に設定
		} else { // EXTERNAL
			ADM2.adm2 = 0x40;	// 外部リファレンスに設定
		}
		if (g_u8PowerManagementMode == PM_SNOOZE_MODE) {
			ADM1.adm1 = 0xE3;	// ハードウェア・トリガ(INTIT)・ウェイト･モード、ワンショットに設定
		} else {
			ADM1.adm1 = 0x20;	// ソフトウェア・トリガ・モード、ワンショットに設定
		}
		ADUL.adul = g_u8ADUL;
		ADLL.adll = g_u8ADLL;
		ADS.ads = u8ADS;		// アナログチャンネルの設定
		delayMicroseconds(5);	// 5 us 待ち
#ifdef WORKAROUND_READ_MODIFY_WRITE
		SBI(SFR_ADM0, SFR_BIT_ADCE);	// A/Dコンパレータを有効に設定
#else /* WORKAROUND_READ_MODIFY_WRITE */
				ADCE = 1;			// A/Dコンパレータを有効に設定
#endif
		if (g_u8PowerManagementMode == PM_SNOOZE_MODE) {
#ifdef WORKAROUND_READ_MODIFY_WRITE
			CBI(SFR_MK1H, 0);	// INTADの割り込み許可
			ADM2.adm2 |= 0x04;	// SNOOZEモードの設定
#else /* WORKAROUND_READ_MODIFY_WRITE */
					ADMK = 0;			// INTADの割り込み許可
					ADM2.adm2 |= 0x04;// SNOOZEモードの設定
#endif
			while (g_bAdcInterruptFlag == false) {
				enterPowerManagementMode(0xFFFFFFFF);	// A/Dコンバート待ち
			}
			ADM2.adm2 &= ~0x04;	// SNOOZEモードの設定解除
			g_bAdcInterruptFlag = false;
			SBI(SFR_MK1H, 0);		// INTADの割り込み禁止
		} else {
			delayMicroseconds(1);	// 1 us 待ち
#ifdef WORKAROUND_READ_MODIFY_WRITE
			SBI(SFR_ADM0, SFR_BIT_ADCS);	// A/Dコンバータの開始
#else /* WORKAROUND_READ_MODIFY_WRITE */
					ADCS = 1;		// A/Dコンバータの開始
#endif
			while (ADIF == 0)
				;	// A/Dコンバート待ち
		}
		s16Result = (ADCR.adcr >> 6);	// A/Dコンバート結果の取得

#ifdef WORKAROUND_READ_MODIFY_WRITE
		CBI(SFR_IF1H, 0);		// INTADの割り込みフラグのクリア
		CBI(SFR_ADM0, SFR_BIT_ADCE);		// A/Dコンパレータを無効に設定
#else /* WORKAROUND_READ_MODIFY_WRITE */
				ADMK = 1;			// INTADの割り込み禁止
				ADIF = 0;// INTADの割り込みフラグのクリア
				ADCE = 0;// A/Dコンパレータを無効に設定
				ADCEN = 0;// A/Dコンバータのクロック供給停止
#endif
	}

	return s16Result;
}

/**
 * SoftwarePWMの次回割り込みタイミングの設定処理
 *
 * @return なし
 *
 * @attention なし
 ***************************************************************************/
void _softwarePWM(void) {
	SwPwm* p;
	for (p = (SwPwm*) &g_SwPwm[0]; p < (SwPwm*) &g_SwPwm[NUM_SWPWM_PINS]; p++) {
		if (p->valid) {
			if (g_u8SwPwmTicksCount == PWM_MIN) {
				p->value = p->newValue;
			}
			if (g_u8SwPwmTicksCount == p->value) {
				*p->port &= ~p->mask;	// 出力をLOWに設定
			} else if (g_u8SwPwmTicksCount == PWM_MIN) {
				*p->port |= p->mask;	// 出力をHIGHに設定
			}
		}
	}
	if (++g_u8SwPwmTicksCount >= PWM_MAX) {
		g_u8SwPwmTicksCount = PWM_MIN;
	};
}

INTERRUPT void adc_interrupt(void) {
	g_bAdcInterruptFlag = true;
}

INTERRUPT void tm06_interrupt(void) {
	_softwarePWM();
}

#endif/*__RL78__*/
