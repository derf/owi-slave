#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

/*
 * Onewire iButton / SmartButton slave.
 * Has the 64bit ID set below
 * (corresponds to <https://wiki.chaosdorf.de/images/f/fc/Smartbutton.jpg>).
 *
 * Only supports non-overdrive READ ROM. Does not hold any data.
 *
 * Tested and working with a DS2482. Should mostly adhere to the standard,
 * but nothing is guaranteed.
 *
 * Any unexpected input (SKIP ROM, overdriv, ...) may cause a hangup requiring
 * an AVR reset.
 */

#define ADDR1 0xC4
#define ADDR2 0x00
#define ADDR3 0x00
#define ADDR4 0x09
#define ADDR5 0x7d
#define ADDR6 0x79
#define ADDR7 0x04
#define ADDR8 0x01

// HCNT: r30 (ZL), r31 (ZH)
// LCNT: r28 (YL), r29 (YH)

#define LCNTH GPIOR2
#define LCNTL GPIOR1
#define HCNTL GPIOR0

#define CNT USIDR
#define LASTCMD OCR0B
#define BUF OCR0A
#define POS OCR1A
#define APOS OCR1B
#define BYTE EEDR

int main (void)
{
	/* watchdog reset after ~4 seconds */
	MCUSR &= ~_BV(WDRF);
	WDTCSR = _BV(WDCE) | _BV(WDE);
	WDTCSR = _BV(WDE) | _BV(WDP3);

	MCUCR = _BV(ISC10);
	GIMSK = _BV(INT1);

	ACSR |= _BV(ACD);

	POS = 0;
	APOS = 0;
	DDRD = 0;
	PORTD = 0;

	sei();

	asm volatile ("ldi r28, 0");
	asm volatile ("ldi r29, 0");
	asm volatile ("ldi r30, 0");
	asm volatile ("ldi r31, 0");

	// 1us
	while (1) { // 1c
		asm volatile ("inc r30"); // 1c
		asm volatile ("adiw r28, 1"); // 2c
		asm volatile ("wdr"); // 1c
	}

	return 0;
}

ISR(INT1_vect)
{
	if (PIND & _BV(PD3)) {

		asm("out 0x15, r29"); // LCNTH
		asm("out 0x14, r28"); // LCNTL
		asm("out 0x13, r30"); // HCNTL

		// > 256us - reset
		if (LCNTH > 0) {
			DDRD = _BV(PD3);
			// 120us
			for (CNT = 0; CNT < 120; CNT++) {
				asm volatile ("wdr");
			}
			DDRD = 0;
			asm volatile ("wdr");
			EIFR |= _BV(INTF1);
		}
		// ~60us - write 0
		else if (LCNTL > 6) {
			if (!LASTCMD)
				POS++;
		}
		// < ~15us - write 1 OR read
		else {
			if (!LASTCMD) {
				BUF |= _BV(POS);
				POS++;
			}
		}
		if (!LASTCMD && (POS == 7)) {
			LASTCMD = BUF;
			POS = 1;
			APOS = 0;
			BYTE = ~ADDR8;
			MCUCR = _BV(ISC11);
			EIFR |= _BV(INTF1);
		}


		asm volatile ("ldi r30, 1"); // HCNTL = 1
	}
	else {
		if (LASTCMD == 0x33) {

			if (BYTE & POS) {

				DDRD = _BV(PD3);
				// 15us
				for (CNT = 0; CNT < 18; CNT++) {
					asm volatile ("wdr");
				}
				DDRD = 0;
				asm volatile ("wdr");
				EIFR |= _BV(INTF1);
			}
			if (POS != 0x80) {
				POS <<= 1;
			}
			else {
				APOS++;
				POS = 1;
				if (APOS == 1)
					BYTE = ~ADDR7;
				else if (APOS == 2)
					BYTE = ~ADDR6;
				else if (APOS == 3)
					BYTE = ~ADDR5;
				else if (APOS == 4)
					BYTE = ~ADDR4;
				else if (APOS == 5)
					BYTE = ~ADDR3;
				else if (APOS == 6)
					BYTE = ~ADDR2;
				else if (APOS == 7)
					BYTE = ~ADDR1;

				else if (APOS == 8) {
					LASTCMD = 0;
					MCUCR = _BV(ISC10);
					POS = 0;
				}
			}
		}
		asm volatile ("ldi r29, 0"); // LCNTH = 0
		asm volatile ("ldi r28, 1"); // LCNTL = 1

	}

}
