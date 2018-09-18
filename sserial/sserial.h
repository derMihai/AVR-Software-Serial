#ifndef SSERIAL_H
#define SSERIAL_H

#include <inttypes.h>

#define SSERIAL_DBG // uncomment for debugging

enum ss_baud {
	BAUD_100,
	BAUD_300,
	BAUD_600,
	BAUD_1200,
	BAUD_2400,
	BAUD_4800,
	BAUD_9600,
	BAUD_14400,
	BAUD_19200,
	BAUD_38400,
	BAUD_57600,
//	BAUD_115200
};

/*
 * rx/tx buffering
 */
#define SSBUFF_RX_SZ 16
#define SSBUFF_TX_SZ 16

class SSerial {
public:
	/*
	 * Pass the pins in hex using the following pattern:
	 * 0x<port><pin>
	 *
	 * Example: port D pin 6 for rx, port B pin 5 for tx
	 * SSerial s(0xD6, 0xB5);
	 *
	 * Preconditions:
	 * 	- the rx pin has to be a valid PCINT pin.
	 */
	SSerial(uint8_t rx, uint8_t tx);

	/*
	 * Starts the software serial using predefined baudrates.
	 */
	void start(ss_baud baud);

	/*
	 * Stop the software serial.
	 */
	void stop(void);

	/*
	 * Sends a byte. If the tx buffer is full, it blocks untill a byte gets free.
	 *
	 * Preconditions:
	 * 	- the software serial was previously started, otherwise behaviour
	 * 	undefined.
	 */
	void put(char data);

	/*
	 * Reads a byte from the software serial.
	 *
	 * returns -1 if no data available.
	 *
	 * Preconditions:
	 * 	- the software serial was previously started, otherwise behaviour
	 * 	undefined.
	 */
	int16_t get(void);

private:
	uint8_t _rx;
	uint8_t _tx;

};

#endif /* SSERIAL_H */

