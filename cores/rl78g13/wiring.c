/*
  wiring.c - Partial implementation of the Wiring API for the ATmega8.
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

  $Id$
*/
/*
 * Modified 15 July 2014 by Nozomu Fujita for GR-SAKURA
 * Modified  4 Mar  2017 by Yuuki Okamiya for RL78/G13
*/

#include "wiring_private.h"
#ifdef __RL78__
#include "utilities.h"
#endif

#ifndef __RL78__
// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)
#else  /*__RL78__*/
volatile unsigned long g_u32timer_millis = 0;	//!< インターバルタイマ変数
volatile unsigned long g_u32delay_timer  = 0;	//!< delay() 用タイマ変数
volatile unsigned long g_timer05_overflow_count = 0;//

#endif /*__RL78__*/

#ifndef __RL78__
volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}
#else /*__RL78__*/

#endif/*__RL78__*/

unsigned long millis()
{
#ifndef __RL78__
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;

	return m;
#else /*__RL78__*/
	unsigned long u32ms;

	byte _psw = PSW.psw;
	noInterrupts();
	u32ms = g_u32timer_millis;
	PSW.psw = _psw;

	return u32ms;
#endif/*__RL78__*/
}

unsigned long micros() {
#ifndef __RL78__
	unsigned long m;
	uint8_t oldSREG = SREG, t;

	cli();
	m = timer0_overflow_count;
#if defined(TCNT0)
	t = TCNT0;
#elif defined(TCNT0L)
	t = TCNT0L;
#else
	#error TIMER 0 not defined
#endif


#ifdef TIFR0
	if ((TIFR0 & _BV(TOV0)) && (t < 255))
		m++;
#else
	if ((TIFR & _BV(TOV0)) && (t < 255))
		m++;
#endif

	SREG = oldSREG;

	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
#else /*__RL78__*/
    unsigned long a;
    unsigned long m;
    uint16_t t;
    boolean ov0,ov1;

#ifdef USE_RTOS
    u32us = (unsigned long)xTaskGetTickCount() / portTICK_RATE_MS * 1000;
#else

    if (g_u8OperationClockMode == CLK_HIGH_SPEED_MODE) {
        // 割り込み禁止状態で
        if (isNoInterrupts()) {
            // TCR05.tcr05 を参照する前の g_timer05_overflow_count の値
            m = g_timer05_overflow_count;
            // TCR05.tcr05 を参照する直前でオーバーフローしてるか?
            ov0 = TMIF05;
            // TCR05.tcr05 の値
            t = TCR05.tcr05;
            // TCR05.tcr05 を参照した直前でオーバーフローしてるか?
            ov1 = TMIF05;

            if (!ov0 && ov1) {
                // TCR05.tcr05 を参照した付近でオーバーフローしたのであれば、
                // t の値は捨てて TDR の初期値を代入し、オーバーフローの補正を行う
                t = INTERVAL_MICRO_TDR;
                m++;
            } else if (ov0) {
                // タイマーが最初っからオーバーフローしてるのであれば、g_timer05_overflow_count の値の補正を行う
                m++;
            }
        // 割り込み許可状態で
        } else {
            // TCR05.tcr05 を参照する直前の g_timer05_overflow_count の値
            a = g_timer05_overflow_count;
            // TCR05.tcr05 の値
            t = TCR05.tcr05;
            // TCR05.tcr05 を参照した直後の g_timer05_overflow_count の値
            m = g_timer05_overflow_count;

            if (a != m) {
                // TCR05.tcr05 を参照する直前と直後の g_timer05_overflow_count の値が
                // 異なっているのであれば、これはどう考えても TCR05.tcr05 の値を参照した付近で
                // インターバルのタイミングが発生してたということなので、t に格納されてる値は捨てて、
                // TCR05.tcr05 の値は TDR の値に設定して問題ない
                t = INTERVAL_MICRO_TDR;
            } else if (t == INTERVAL_MICRO_TDR) {
                // TCR05 がオーバーフローを起こし、割り込みが発生していれば割り込みの処理で
                // 32クロック以上掛かるので、TCR05.tcr05 の値は TDR05 の値より -1 以上
                // 小さくなっている筈。それがなく、TCR05.tcr05 の値が TDR05 と等しいということは
                // 割り込み処理がまだ行われてないということなので補正を行う
                m++;
            }
        }
        m = m * MICROSECONDS_PER_TIMER05_OVERFLOW + (INTERVAL_MICRO_TDR - t);
    }
    else {
        // ミリ秒 x 1000;
        m = millis() * 1000;
    }
#endif

    return m;
#endif/*__RL78__*/
}

void delay(unsigned long ms)
{
#ifndef __RL78__
	uint16_t start = (uint16_t)micros();

	while (ms > 0) {
		yield();
		if (((uint16_t)micros() - start) >= 1000) {
			ms--;
			start += 1000;
		}
	}
#else /*__RL78__*/

	if (g_u8PowerManagementMode == PM_NORMAL_MODE) {
		uint16_t start = (uint16_t)micros();

		while (ms > 0) {
			if (((uint16_t)micros() - start) >= 1000) {
				ms--;
				start += 1000;
			}
		}
	} else {
		enterPowerManagementMode(ms);
	}
#endif/*__RL78__*/
}

/* Delay for the given number of microseconds.  Assumes a 8 or 16 MHz clock. */
void delayMicroseconds(unsigned int us)
{
#ifndef __RL78__
	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
#if F_CPU >= 20000000L
	// for the 20 MHz clock on rare Arduino boards

	// for a one-microsecond delay, simply wait 2 cycle and return. The overhead
	// of the function call yields a delay of exactly a one microsecond.
	__asm__ __volatile__ (
		"nop" "\n\t"
		"nop"); //just waiting 2 cycle
	if (--us == 0)
		return;

	// the following loop takes a 1/5 of a microsecond (4 cycles)
	// per iteration, so execute it five times for each microsecond of
	// delay requested.
	us = (us<<2) + us; // x5 us

	// account for the time taken in the preceeding commands.
	us -= 2;

#elif F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;
#else
	// for the 8 MHz internal clock on the ATmega168

	// for a one- or two-microsecond delay, simply return.  the overhead of
	// the function calls takes more than two microseconds.  can't just
	// subtract two, since us is unsigned; we'd overflow.
	if (--us == 0)
		return;
	if (--us == 0)
		return;

	// the following loop takes half of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	us <<= 1;

	// partially compensate for the time taken by the preceeding commands.
	// we can't subtract any more than this or we'd overflow w/ small delays.
	us--;
#endif

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
#else /*__RL78__*/
	if (g_u8PowerManagementMode == PM_NORMAL_MODE) {
		unsigned long s, w, d;
		s = micros();
		w = us;
		d = 0;
		while (w > d) {
			d = micros() - s;
		}
	} else {
		enterPowerManagementMode(us / 1000);
	}
#endif/*__RL78__*/
}

void init()
{
#ifndef __RL78__
	// this needs to be called before setup() or some functions won't
	// work there
	sei();

	// on the ATmega168, timer 0 is also used for fast hardware pwm
	// (using phase-correct PWM would mean that timer 0 overflowed half as often
	// resulting in different millis() behavior on the ATmega8 and ATmega168)
#if defined(TCCR0A) && defined(WGM01)
	sbi(TCCR0A, WGM01);
	sbi(TCCR0A, WGM00);
#endif  

	// set timer 0 prescale factor to 64
#if defined(__AVR_ATmega128__)
	// CPU specific: different values for the ATmega128
	sbi(TCCR0, CS02);
#elif defined(TCCR0) && defined(CS01) && defined(CS00)
	// this combination is for the standard atmega8
	sbi(TCCR0, CS01);
	sbi(TCCR0, CS00);
#elif defined(TCCR0B) && defined(CS01) && defined(CS00)
	// this combination is for the standard 168/328/1280/2560
	sbi(TCCR0B, CS01);
	sbi(TCCR0B, CS00);
#elif defined(TCCR0A) && defined(CS01) && defined(CS00)
	// this combination is for the __AVR_ATmega645__ series
	sbi(TCCR0A, CS01);
	sbi(TCCR0A, CS00);
#else
	#error Timer 0 prescale factor 64 not set correctly
#endif

	// enable timer 0 overflow interrupt
#if defined(TIMSK) && defined(TOIE0)
	sbi(TIMSK, TOIE0);
#elif defined(TIMSK0) && defined(TOIE0)
	sbi(TIMSK0, TOIE0);
#else
	#error	Timer 0 overflow interrupt not set correctly
#endif

	// timers 1 and 2 are used for phase-correct hardware pwm
	// this is better for motors as it ensures an even waveform
	// note, however, that fast pwm mode can achieve a frequency of up
	// 8 MHz (with a 16 MHz clock) at 50% duty cycle

#if defined(TCCR1B) && defined(CS11) && defined(CS10)
	TCCR1B = 0;

	// set timer 1 prescale factor to 64
	sbi(TCCR1B, CS11);
#if F_CPU >= 8000000L
	sbi(TCCR1B, CS10);
#endif
#elif defined(TCCR1) && defined(CS11) && defined(CS10)
	sbi(TCCR1, CS11);
#if F_CPU >= 8000000L
	sbi(TCCR1, CS10);
#endif
#endif
	// put timer 1 in 8-bit phase correct pwm mode
#if defined(TCCR1A) && defined(WGM10)
	sbi(TCCR1A, WGM10);
#elif defined(TCCR1)
	#warning this needs to be finished
#endif

	// set timer 2 prescale factor to 64
#if defined(TCCR2) && defined(CS22)
	sbi(TCCR2, CS22);
#elif defined(TCCR2B) && defined(CS22)
	sbi(TCCR2B, CS22);
#else
	#warning Timer 2 not finished (may not be present on this CPU)
#endif

	// configure timer 2 for phase correct pwm (8-bit)
#if defined(TCCR2) && defined(WGM20)
	sbi(TCCR2, WGM20);
#elif defined(TCCR2A) && defined(WGM20)
	sbi(TCCR2A, WGM20);
#else
	#warning Timer 2 not finished (may not be present on this CPU)
#endif

#if defined(TCCR3B) && defined(CS31) && defined(WGM30)
	sbi(TCCR3B, CS31);		// set timer 3 prescale factor to 64
	sbi(TCCR3B, CS30);
	sbi(TCCR3A, WGM30);		// put timer 3 in 8-bit phase correct pwm mode
#endif

#if defined(TCCR4A) && defined(TCCR4B) && defined(TCCR4D) /* beginning of timer4 block for 32U4 and similar */
	sbi(TCCR4B, CS42);		// set timer4 prescale factor to 64
	sbi(TCCR4B, CS41);
	sbi(TCCR4B, CS40);
	sbi(TCCR4D, WGM40);		// put timer 4 in phase- and frequency-correct PWM mode
	sbi(TCCR4A, PWM4A);		// enable PWM mode for comparator OCR4A
	sbi(TCCR4C, PWM4D);		// enable PWM mode for comparator OCR4D
#else /* beginning of timer4 block for ATMEGA1280 and ATMEGA2560 */
#if defined(TCCR4B) && defined(CS41) && defined(WGM40)
	sbi(TCCR4B, CS41);		// set timer 4 prescale factor to 64
	sbi(TCCR4B, CS40);
	sbi(TCCR4A, WGM40);		// put timer 4 in 8-bit phase correct pwm mode
#endif
#endif /* end timer4 block for ATMEGA1280/2560 and similar */	

#if defined(TCCR5B) && defined(CS51) && defined(WGM50)
	sbi(TCCR5B, CS51);		// set timer 5 prescale factor to 64
	sbi(TCCR5B, CS50);
	sbi(TCCR5A, WGM50);		// put timer 5 in 8-bit phase correct pwm mode
#endif

#if defined(ADCSRA)
	// set a2d prescale factor to 128
	// 16 MHz / 128 = 125 KHz, inside the desired 50-200 KHz range.
	// XXX: this will not work properly for other clock speeds, and
	// this code should use F_CPU to determine the prescale factor.
	sbi(ADCSRA, ADPS2);
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);

	// enable a2d conversions
	sbi(ADCSRA, ADEN);
#endif

	// the bootloader connects pins 0 and 1 to the USART; disconnect them
	// here so they can be used as normal digital i/o; they will be
	// reconnected in Serial.begin()
#if defined(UCSRB)
	UCSRB = 0;
#elif defined(UCSR0B)
	UCSR0B = 0;
#endif
#else /*__RL78__*/

	// リセット要因の保存
	g_u8ResetFlag = RESF.resf;

	// 割り込みの禁止
	noInterrupts();

#if defined(__RL78__)
	// 周辺I/Oリダイレクション・レジスタの設定
	PIOR.pior = 0x09;	// PCLBUZ0とTI0x/TO0xをリダイレクション
	RPECTL.rpectl = 0x80;
#elif defined(__RL78___PROTOTYPE)
	// ポート・モード・コントロール・レジスタの設定
	PMC0.pmc0 = 0xF3;
#endif

#ifdef GRCOTTON
	PMC0.pmc0 = 0xF0;
#endif

#if (RTC_CLK_SOURCE == CLK_SOURCE_XT1)
	unsigned long i;
	CMC.cmc   = 0x10;	// クロック動作モード制御レジスタの設定（X1発振回路=未使用、XT1発振回路=使用、XT1低消費発振）
	MSTOP     = 1;		// X1発信回路の発振停止
	MCM0      = 0;		// メイン・システム・クロックに高速オンチップ・オシレータ・クロックを選択
	XTSTOP    = 0;		// XT1発振回路の発振開始
	for (i = 0; i < WAIT_XT1_CLOCK; i++) {
		_NOP();			// XT1発振回路の安定化待ち
	}
	OSMC.osmc = 0x00;	// 周辺機器へのサブシステム・クロックの供給許可、RTC/ITのクロックにXT1発振回路を選択
#elif (RTC_CLK_SOURCE == CLK_SOURCE_FIL)
	CMC.cmc   = 0x10;	// クロック動作モード制御レジスタの設定（X1発振回路=未使用、XT1発振回路=使用、XT1低消費発振）
	MSTOP     = 1;		// X1発信回路の発振停止
	XTSTOP    = 1;		// XT1発振回路の発振停止
	MCM0      = 0;		// メイン・システム・クロックに高速オンチップ・オシレータ・クロックを選択
	OSMC.osmc = 0x10;	// 周辺機器へのサブシステム・クロックの供給許可、RTC/ITのクロックに低速オンチップ・オシレータを選択
#endif
	HIOSTOP = 0;	// 高速オンチップ・オシレータの動作開始
	CSS     = 0;	// CPU/周辺ハードウェア・クロックににメイン・システム・クロックを選択
	while (CLS != 0);

	RTCEN      = 1;			// RTCのクロック供給開始
	ITMC.itmc  = 0x0000;	// インターバル・タイマ・コントロール・レジスタの初期化
	ITMK       = 1;			// INTIT割り込みの禁止
	ITPR1      = 1;			// INTIT割り込みの優先度設定
	ITPR0      = 1;
	ITMC.itmc  = INTERVAL;	// 周期の設定

	TAU0EN    = 1;				// タイマ・アレイ・ユニットにクロック供給開始
	TPS0.tps0 = TIMER_CLOCK;	// タイマ・クロック周波数を設定
	TT0.tt0     |= 0x0020;			// マイクロ秒タイマの動作停止
	TMMK05     = 1;			// INTTM05割り込みマスク解除
	TMIF05     = 0;			// INTTM05割り込み要因のクリア
	TMPR005     = 0;			// INTTM05割り込み要因のクリア
	TMPR105     = 0;			// INTTM05割り込み要因のクリア
	TMR05.tmr05  = INTERVAL_MICRO_MODE;	// マイクロ秒タイマの動作モード設定
	TDR05.tdr05  = INTERVAL_MICRO_TDR;	// マイクロ秒タイマの周期設定
    TOM0.tom0 &= ~0x0020;
    TOL0.tol0 &= ~0x0020;
	TO0.to0     &= ~0x0020;			// マイクロ秒タイマの出力設定
	TOE0.toe0   &= ~0x0020;			// マイクロ秒タイマの出力許可設定

	ITIF       = 0;			// INTIT割り込み要因のクリア
	ITMK       = 0;			// INTIT割り込みの許可
	ITMC.itmc |= 0x8000;	// インターバル・タイマの開始
	TMIF05     = 0;			// INTTM05割り込み要因のクリア
	TMMK05     = 0;			// INTTM05割り込みマスク解除
	TS0.ts0   |= 0x0020;	// マイクロ秒タイマの開始
	g_timer05_overflow_count = 0; //
	g_u32timer_millis = 0;	// カウンタの初期化

	SBI2(SFR2_PER0, SFR2_BIT_ADCEN);	// A/Dコンバータにクロック供給開始
	SBI(SFR_MK1H, 0);		// INTADの割り込み禁止
	CBI(SFR_IF1H, 0);		// INTADの割り込みフラグのクリア
	SBI(SFR_PR11H, 1);		// INTADの割り込み優先順位の設定
	SBI(SFR_PR01H, 1);

	// 割り込みの許可
	interrupts();

#endif/*__RL78__*/
}

/**
 * インターバル･タイマの停止
 *
 * @return 返り値なし
 *
 * @attention なし
 ***************************************************************************/
void stop_interval_timer()
{
	ITMC.itmc &= ~0x8000;	// インターバル・タイマの停止
	ITMK       = 1;			// INTIT割り込みの禁止
	ITIF       = 0;			// INTIT割り込み要因のクリア
}

/**
 * インターバル･タイマ割り込みハンドラ
 *
 * @return 返り値なし
 *
 * @attention なし
 ***************************************************************************/
INTERRUPT void interval_timer()
{
	// 1 msecカウンタのインクリメント
	g_u32timer_millis++;

	if (g_u32delay_timer  != 0) {
		// delay() タイマのデクリメント
		g_u32delay_timer--;
	}

	if (g_fITInterruptFunc != NULL) {
		(*g_fITInterruptFunc)(g_u32timer_millis);
	}
}

INTERRUPT void tm05_interrupt(void)
{
	g_timer05_overflow_count++;
}



