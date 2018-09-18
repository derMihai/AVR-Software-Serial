# AVR Software-UART library

AVR software serial, featuring asynchronous full duplex. Currently only tested on the ATmega328p.

TIM0 is used for timings. 

PCINT triggers the sampling of incoming data, thus the rx pin has to be a valid PCINT pin. Not that all PCINT vectors are defined by this library.

For baudrates up to 19200, input sampling and signal generation is interrupt driven and asynchronous, thus freeing the CPU and supporting baud rates as low as 100 bps (__non-fastmode__). 

For baud rates up to 57600, only half duplex is supported because the CPU is busy while sending/receiving each byte (__fast mode__).

Momentarly only 1 stop bit, no parity, non-reverse signal/bit order mode is supported.

Only MCUs running at 16 Mhz work.

## Getting started

The rx/tx buffer sizes can be adjusted by setting the `SSBUFF_RX_SZ` and `SSBUFF_TX_SZ` constants, thus allowing larger buffers to be handled asynchronously. Note that in fast mode, since the sending is handled in the thread, the tx buffer size doesn't matter.

### Example

```C

#include "sserial/sserial.h"
SSerial ss(0xB3, 0xC0); // port B pin 3 for rx, port C pin 0 for tx

void ss_putstr(char *str)
{
    while (*str)
        ss.put(*str++);
}

int main(void)
{
    int16_t data_in;
    sei();
    ss.start(BAUD_19200);

    ss_putstr((char*)"Restart\n");

    while (1) {
        if ((data_in = ss.get()) != -1) {
            ss.put(data_in);
        }
    }

    return 0;
}

```

#TODOs

* Support higher baudrates, probably up to 115200
* Support different CPU speeds
* Support parity bit and 2 stop bits
* Optimize the timings (maybe some assembly)
* Full duplex in fast mode