/*MIT License

Copyright (c) 2018 Mihai Renea

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef SS_MACROS_H
#define SS_MACROS_H

#include <avr/io.h>
#include "sserial.h"

/*
 * Gets the port/pin value from the hex values passed to the constructor
 */
#define hex_get_port(__port_pin) (((__port_pin) >> 4) - 0xB)
#define hex_get_pin(__port_pin) ((__port_pin) & 0x0F)

/*
 * Pull the tx line high/low
 */
#define tx_low(void)		PORTX &= ~tx_pin_msk
#define tx_high(void)		PORTX |= tx_pin_msk

/*
 * read the input line
 */
#define in_high(void)		(PINX & rx_pin_msk)

#define tx_pin_init(void)	*(&DDRB + hex_get_port(_tx) * 3) |= (1 << hex_get_pin(_tx))
#define rx_pin_init(void)	*(&DDRB + hex_get_port(_rx) * 3) &= ~(1 << hex_get_pin(_rx))

#define pcint_init(void)	PCMSKX = rx_pin_msk

/*
 * Clear PCINT flag and enable interrupt.
 */
#define pcint_enable(void)	do {\
	PCIFR |= rx_port_msk;\
	PCICR |= rx_port_msk;\
} while (0)

#define pcint_disable(void) (PCICR &= ~rx_port_msk)

/*
 * clears compare match A/B flag and enables interrupt
 */
#define compa_int_enable(void)	do {\
	TIFR0 |= (1 << OCF0A);\
	TIMSK0 |= (1 << OCIE0A);\
} while (0)

#define compb_int_enable(void) do {\
	TIFR0 |= (1 << OCF0B);\
	TIMSK0 |= (1 << OCIE0B);\
} while (0)

/*
 * disables compare match A/B interrupt
 */
#define compa_int_disable(void)	(TIMSK0 &= ~(1 << OCIE0A))
#define compb_int_disable(void)	(TIMSK0 &= ~(1 << OCIE0B))

/*
 * clear compare match A/B interrupt flag.
 */
#define clr_compa_flag(void)	TIFR0 |= (1 << OCF0A)
#define clr_compb_flag(void)	TIFR0 |= (1 << OCF0B)

/*
 * Wait until compare A/B interrupt flag is set, then clear it.
 */
#define wait_for_compa_flag(void)	do {\
	while (!(TIFR0 & (1 << OCF0A)));\
	clr_compa_flag();\
} while (0)

#define wait_for_compb_flag(void)	do {\
	while (!(TIFR0 & (1 << OCF0B)));\
	clr_compb_flag();\
} while (0)

/*
 * Stop the TIM0.
 */
#define TIM0_stop(void) TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00))

enum encode_action {
	START_BIT,
	SHIFT_IN,
//	PAR_BIT,
//	STOP_BIT
};

/*
 * Timer 0 prescalers
 */
enum tim0_p {
	T0P_1,
	T0P_8,
	T0P_64,
	T0P_256,
	T0P_1024,
};

enum ss_sreg_bit {
	FAST_MODE,
//	OVERFLOW,
//	PAR_CHCK_FAIL,
//	SPURIOUS_BIT
};

#define sreg_set_bit(__bit) (ss_sreg |= (1 << __bit))
#define sreg_clr_bit(__bit) (ss_sreg &= ~(1 << __bit))
#define sreg_bit_isset(__bit) (ss_sreg & (1 << __bit))

/*
 * The semaphore macros are non-blocking and not atomic!
 */
typedef volatile uint8_t semaphore;
#define sem_acquire_nobl(__sem) ((__sem > 0) ? __sem-- : 0)
#define sem_release_nobl(__sem, __s_max) ((__sem < __s_max) ? ++__sem : 0)



/*
 * For debugging only.
 */
#ifdef SSERIAL_DBG
/*
 * Toggle a pin before each input line sampling.
 */
#define tog_dbgpin_sampling_init(void) PORTB |= (1 << PB5)
#define tog_dbgpin_sampling(void) PINB |= (1 << PB5)

#else
#define tog_dbgpin_sampling_init(void) while(0)
#define tog_dbgpin_sampling(void) while(0)
#endif

#endif /* SS_MACROS_H */
