/*
  specific_instructions.h - RX specific functions
  Copyright (c) 2014 Nozomu Fujita.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/*
 * Mar 1st 2017: Modified for RL78 by Yuuki Okamiya
 */

#ifndef _SPECIFIC_INSTRUCTIONS_H_
#define _SPECIFIC_INSTRUCTIONS_H_

#define sei() \
do { \
  __asm __volatile("EI\n"); \
} while (0)

#define cli() \
do { \
  __asm __volatile("DI\n"); \
} while (0)

#define isInterrupts() (PSW.BIT.ie == 1)

#define isNoInterrupts() (PSW.BIT.ie == 0)

#define pushi() \
{ \
  bool _di = isNoInterrupts();

#define popi() \
  if (_di) { \
    cli(); \
  } else { \
    sei(); \
  } \
}

#define CBI(sfr, bit) \
do { \
    __asm __volatile( \
        "        clr1    %u0.%u1 \n" \
        : \
        : "i"(sfr), "i"(bit) \
    ); \
} while (0)
#define SBI(sfr, bit) \
do { \
    __asm __volatile( \
        "        set1    %u0.%u1 \n" \
        : \
        : "i"(sfr), "i"(bit) \
    ); \
} while (0)

#define CBI2(sfr, bit) \
do { \
    __asm __volatile( \
        "        clr1    !%u0.%u1 \n" \
        : \
        : "i"(sfr), "i"(bit) \
    ); \
} while (0)
#define SBI2(sfr, bit) \
do { \
    __asm __volatile( \
        "        set1    !%u0.%u1 \n" \
        : \
        : "i"(sfr), "i"(bit) \
    ); \
} while (0)

#ifdef WORKAROUND_READ_MODIFY_WRITE
#define cbi(psfr, bit) \
{ \
	__asm __volatile( \
		"	movw	hl, %0					\n" \
		"	mov	a, %1					\n" \
		"	xor	a, #0xff				\n" \
		"	push	psw					\n" \
		"	di						\n" \
		"	and	a, [hl]					\n" \
		"	mov	[hl], a					\n" \
		"	pop	psw					\n" \
		: \
		:"r"(psfr), "r"(bit) \
		:"a", "h", "l" \
	); \
}

#define sbi(psfr, bit) \
{ \
	__asm __volatile( \
		"	movw	hl, %0					\n" \
		"	mov	a, %1					\n" \
		"	push	psw					\n" \
		"	di						\n" \
		"	or	a, [hl]					\n" \
		"	mov	[hl], a					\n" \
		"	pop	psw					\n" \
		: \
		:"r"(psfr), "r"(bit) \
		:"a", "h", "l" \
	); \
}
#endif

#endif/*_SPECIFIC_INSTRUCTIONS_H_*/
