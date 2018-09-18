#include "sserial.h"
#include "ss_macros.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>

/*
 * Output compare register A values table
 */
static const uint8_t PROGMEM OCRA_vals[] = {
					// baud
		156 - 1,	// 100
		208 - 1,	// 300
		104 - 1,	// 600
		208 - 1,	// 1200
		104 - 1,	// 2400
		52 - 1, 	// 4800
		208 - 1,	// 9600
		139 - 1,	// 14400
		104 - 1,	// 19200
		52 - 1, 	// 38400
		35 - 1,		// 57600
//		139 - 1,	// 115200
};

/*
 * TIM0 prescaler values table
 */
static const uint8_t PROGMEM TIM0_pre[] = {
									// baud		prescaler
		(1 << CS02) | (1 << CS00),	// 100		1024
		(1 << CS02),				// 300		256
		(1 << CS02),				// 600		256
		(1 << CS01) | (1 << CS00),	// 1200		64
		(1 << CS01) | (1 << CS00),	// 2400		64
		(1 << CS01) | (1 << CS00), 	// 4800		64
		(1 << CS01),				// 9600		8
		(1 << CS01),				// 14400	8
		(1 << CS01),				// 19200	8
		(1 << CS01), 				// 38400	8
		(1 << CS01),				// 57600	8
//		(1 << CS00),				// 115200	1
};

/*
 * Precomputed values for register accesses to save critical CPU cycles
 */
static volatile uint8_t rx_pin_msk;
static volatile uint8_t tx_pin_msk;
static volatile uint8_t rx_port_msk;		// to set the PCINT mask reg

static volatile uint8_t *portx = &PORTB;	// output port address
#define PORTX 	(*(portx))					// output port value

static volatile uint8_t *pinx = &PINB;		// input port address
#define PINX (*(pinx))						// input port value

static volatile uint8_t *pcmskx = &PCMSK0;	// PCINT interrupt mask reg address
#define PCMSKX (*(pcmskx))					// PCINT interrupt mask reg value

/*
 * status register
 */
static volatile uint8_t ss_sreg;

/*
 * circular rx/tx buffers
 */
static volatile char rx_buff[SSBUFF_RX_SZ];
static volatile char tx_buff[SSBUFF_TX_SZ];
/*
 * semaphores for buffers
 */
static semaphore rx_sem = 0;
static semaphore tx_sem = 0;
/*
 * thread indices for buffers
 */
static uint8_t rx_buff_i_thd;
static uint8_t tx_buff_i_thd;
/*
 * ISR indices for buffers
 */
static uint8_t rx_buff_i_isr;
static uint8_t tx_buff_i_isr;

static inline void init_timer(ss_baud baud);
//static void set_prescaler(tim0_p pre);


SSerial::SSerial(uint8_t rx, uint8_t tx)
{
	_rx = rx;
	_tx = tx;

	/*
	 * precompute register access vaues
	 */
	portx += hex_get_port(tx) * 3;
	rx_port_msk = (1 << hex_get_port(rx));
	rx_pin_msk = (1 << hex_get_pin(rx));
	pinx += hex_get_port(rx) * 3;
	pcmskx += hex_get_port(rx);
	tx_pin_msk = (1 << hex_get_pin(tx));
}

void SSerial::start(ss_baud baud)
{
	tog_dbgpin_sampling_init();

	rx_sem = tx_sem = 0;
	rx_buff_i_isr = rx_buff_i_thd = 0;
	tx_buff_i_isr = tx_buff_i_thd = 0;

	ss_sreg = 0;

	if (baud > BAUD_19200) {
		sreg_set_bit(FAST_MODE);
	}

	init_timer(baud);

	rx_pin_init();
	tx_pin_init();

	tx_high(); // pull the line high, as required by the protocol

	pcint_init();
	pcint_enable();
}

void SSerial::stop(void)
{
	TIM0_stop();
	pcint_disable();
	tx_low(); // pull the line low, to signal the device is disconnected
}


void SSerial::put(char data)
{
	if (sreg_bit_isset(FAST_MODE)) {
		uint8_t mask = 1;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			clr_compa_flag();

			// start bit
			wait_for_compa_flag();
			tx_low();

			// data bits
			while (mask) {
				wait_for_compa_flag();

				if (data & mask)
					tx_high();
				else
					tx_low();

				mask <<= 1;
			}
			// stop bit
			wait_for_compa_flag();
			tx_high();
		}
		return;

	} else {
		uint8_t sem_rel; // semaphore released

		do {
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
				if ((sem_rel = sem_release_nobl(tx_sem, SSBUFF_TX_SZ)))
					tx_buff[tx_buff_i_thd] = data;
			}

			/*
			 * ensure interrupts get served
			 */
			asm volatile (
					"NOP \n\t"
					"NOP \n\t"
					"NOP \n\t"
					"NOP \n\t"
					"NOP \n\t"
					"NOP \n\t"
					"NOP \n\t"
					"NOP \n\t"
			::);

		} while (!sem_rel);

		tx_buff_i_thd = (tx_buff_i_thd + 1) % SSBUFF_TX_SZ;
		compa_int_enable();
	}
}

int16_t SSerial::get(void)
{
	int16_t retval = -1;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (sem_acquire_nobl(rx_sem)) {
			retval = rx_buff[rx_buff_i_thd];
			rx_buff_i_thd = (rx_buff_i_thd + 1) % SSBUFF_RX_SZ;
		}
	}

	return retval;
}

/*
 * for tx (non-fastmode)
 */
ISR(TIMER0_COMPA_vect)
{
	static encode_action action = START_BIT;
	static char data_out;
	static uint8_t mask = 1;

	switch(action) {

	case START_BIT:
		if (!sem_acquire_nobl(tx_sem)) {
			compa_int_disable();
			break;
		}

		tx_low();
		data_out = tx_buff[tx_buff_i_isr];
		action = SHIFT_IN;
		break;

	case SHIFT_IN:
		if (mask) {
			if (mask & data_out)
				tx_high();
			else
				tx_low();

			mask <<= 1;

		} else {
			tx_high(); // stop bit
			mask = 1;
			action = START_BIT;
			tx_buff_i_isr = (tx_buff_i_isr + 1) % SSBUFF_TX_SZ;
		}
		break;
	}
}

/*
 * for rx (non-fastmode)
 */
ISR(TIMER0_COMPB_vect)
{
	static encode_action action = START_BIT;
	static uint8_t mask = 1;
	static unsigned char data_in = 0;

	tog_dbgpin_sampling();
	uint8_t in_pin_high = in_high();

	switch (action) {

	case START_BIT:
		/*
		 * If the rx line is already high, it is considered a spurious start
		 * bit and thus ignored.
		 */
		if (in_pin_high) {
			compb_int_disable();
			pcint_enable();
			return;
		}

		action = SHIFT_IN;
		break;

	case SHIFT_IN:
		if (mask) {
			if (in_pin_high) {
				data_in |= mask;
			}
			mask <<= 1;

		} else {
			/*
			 * If the stop bit is low, the byte is ignored.
			 */
			if (in_pin_high) {
				if (sem_release_nobl(rx_sem, SSBUFF_RX_SZ)) {
					rx_buff[rx_buff_i_isr] = data_in;
					rx_buff_i_isr = (rx_buff_i_isr + 1) % SSBUFF_RX_SZ;
				}
			}

			mask = 1;
			action = START_BIT;
			data_in = 0;
			compb_int_disable();
			pcint_enable();
		}
		break;
	}
}

static inline void init_timer(ss_baud baud)
{
	TCCR0A = (1 << WGM01); // clear counter on compare match
	OCR0A = pgm_read_byte(&OCRA_vals[baud]);
	TCCR0B = pgm_read_byte(&TIM0_pre[baud]);
}

/*
 * Triggered by the rx start bit.
 */
ISR(PCINT0_vect)
{
	uint16_t offset = TCNT0; // the time the stop bit arrived, relative to the counter
	tog_dbgpin_sampling();
	uint8_t in_pin_high = in_high();

	/*
	 * Add a quarter period to the offset. This will be the output compare B
	 * value, which will trigger input line sampling.
	 */
	offset += OCR0A >> 2;
	// 16bit modulo is expensive, use substraction instead
	if (offset > OCR0A)
			offset -= OCR0A;

	OCR0B = offset;

	/*
	 * If the interrupt was triggered by a rising edge or the input line has risen
	 * up again so fast, the start bit is ignored.
	 */
	if (in_pin_high) {
		return;
	}

	/*
	 * In fast mode, do the whole sampling in this ISR.
	 */
	if (sreg_bit_isset(FAST_MODE)) {
		clr_compb_flag();
		uint8_t mask = 1;
		unsigned char data_in = 0;

		wait_for_compb_flag();
		tog_dbgpin_sampling();
		/*
		 * If the rx line is already high, it is considered a spurious start
		 * bit and thus ignored.
		 */
		if (in_high()) {
			pcint_enable();
			return;

		}

		// data bits
		while (mask) {
			wait_for_compb_flag();

			tog_dbgpin_sampling();
			if (in_high())
				data_in |= mask;

			mask <<= 1;
		}

		// stop bit
		wait_for_compb_flag();
		tog_dbgpin_sampling();
		/*
		 * If the stop bit is low, the byte is ignored.
		 */
		if (in_high()) {
			if (sem_release_nobl(rx_sem, SSBUFF_RX_SZ)) {
				rx_buff[rx_buff_i_isr] = data_in;
				rx_buff_i_isr = (rx_buff_i_isr + 1) % SSBUFF_RX_SZ;
			}
		}

		pcint_enable();

	} else {
		pcint_disable();
		compb_int_enable();
	}

}

ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect)); // @suppress("Unused function declaration")
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect)); // @suppress("Unused function declaration")

