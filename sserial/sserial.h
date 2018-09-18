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

