#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

/*
 * Onewire iButton / SmartButton slave.
 * Has the 64bit ID set below
 * (corresponds to <https://wiki.chaosdorf.de/images/f/fc/Smartbutton.jpg>).
 * Note: The bytes are sent in reversed order. This has also been observed
 * on off-the-shelf smartbuttons / iButtons.
 *
 * Only supports non-overdrive READ ROM. Does not hold any data.
 * SEARCH ROM Support is work in progress. Command byte readout is pretty
 * broken right now.
 *
 * Tested and working with a DS2482. Should mostly adhere to the standard,
 * but nothing is guaranteed.
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
	WDTCR = _BV(WDCE) | _BV(WDE);
	WDTCR = _BV(WDE) | _BV(WDP3);

	MCUCR = _BV(ISC10);
	GIMSK = _BV(INT1);

	ACSR |= _BV(ACD);

	PRR = _BV(PRUSI) | _BV(PRUSART);

	POS = 0;
	APOS = 0;
	DDRD = 0;
	PORTD = 0;

	sei();

	asm volatile ("ldi r28, 0");
	asm volatile ("ldi r29, 0");
	asm volatile ("ldi r30, 0");
	asm volatile ("ldi r31, 0");

	// 1us per cycle
loop:
	asm volatile ("inc r30"); // 1c
	asm volatile ("adiw r28, 1"); // 2c
	asm volatile ("wdr"); // 1c
	asm volatile ("nop"); // 1c
	asm volatile ("nop"); // 1c
	asm volatile ("nop"); // 1c
	goto loop;

	return 0;
}

ISR(INT1_vect)
{
	// overhead: 19c (2.4us)
	if (PIND & _BV(PD3)) {

		asm volatile ("out %0, r29" : : "M" (_SFR_IO_ADDR(LCNTH)));
		asm volatile ("out %0, r28" : : "M" (_SFR_IO_ADDR(LCNTL)));
		asm volatile ("out %0, r30" : : "M" (_SFR_IO_ADDR(HCNTL)));

		// > 256us - reset
		if (LCNTH > 0) {
			DDRD = _BV(PD3);

			// 120us loop - r31 / r30 need not be preserved
			asm volatile ("ldi r31, 0");
			asm volatile ("ldi r30, 120"); // Z = 120
			asm volatile ("wdr"); // <-----
			asm volatile ("wdr");
			asm volatile ("wdr");
			asm volatile ("sbiw r30, 1");
			asm volatile ("cp r30, r1");
			asm volatile ("brne .-12"); // -^

			DDRD = 0;
			LASTCMD = 0;
			BUF = 0;
			POS = 0;
			asm volatile ("wdr");
			EIFR |= _BV(INTF1);
		}
		// ~60us - write 0
		else if (LCNTL > 15) {
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
			EEAR = 0;
			EIFR |= _BV(INTF1);
		}


		asm volatile ("ldi r30, 1"); // HCNTL = 1
	}
	else {
		if (LASTCMD == 0x33) {

			if (BYTE & POS) {

				DDRD = _BV(PD3);

				// 15us loop - r31 / r28 need not be preserved
				asm volatile ("ldi r31, 0");
				asm volatile ("ldi r28, 15"); // Z = 15
				asm volatile ("wdr"); // <-----
				asm volatile ("wdr");
				asm volatile ("wdr");
				asm volatile ("sbiw r28, 1");
				asm volatile ("cp r28, r1");
				asm volatile ("brne .-12"); // -^

				DDRD = 0;
				asm volatile ("wdr");
				EIFR |= _BV(INTF1);
			}
			if (POS != 0x80) {
				POS <<= 1;
			}
			else {
				POS = 1;
				// store APOS in r28 (written back at the end)
				// and BYTE in r29 (also written back at the end)
				asm volatile ("in r28, %0" : : "M" (_SFR_IO_ADDR(APOS)));
				asm volatile ("inc r28"); // APOS++
				asm volatile ("cpi r28, 1");
				asm volatile ("brne .+2");
				asm volatile ("ldi r29, %0" : : "i" (~ADDR7));
				asm volatile ("cpi r28, 2");
				asm volatile ("brne .+2");
				asm volatile ("ldi r29, %0" : : "i" (~ADDR6));
				asm volatile ("cpi r28, 3");
				asm volatile ("brne .+2");
				asm volatile ("ldi r29, %0" : : "i" (~ADDR5));
				asm volatile ("cpi r28, 4");
				asm volatile ("brne .+2");
				asm volatile ("ldi r29, %0" : : "i" (~ADDR4));
				asm volatile ("cpi r28, 5");
				asm volatile ("brne .+2");
				asm volatile ("ldi r29, %0" : : "i" (~ADDR3));
				asm volatile ("cpi r28, 6");
				asm volatile ("brne .+2");
				asm volatile ("ldi r29, %0" : : "i" (~ADDR2));
				asm volatile ("cpi r28, 7");
				asm volatile ("brne .+2");
				asm volatile ("ldi r29, %0" : : "i" (~ADDR1));
				asm volatile ("out %0, r29" : : "M" (_SFR_IO_ADDR(BYTE)));

				asm volatile ("cpi r28, 8");
				asm volatile ("brne .+4");
				asm volatile ("out %0, r1" : : "M" (_SFR_IO_ADDR(LASTCMD)));
				asm volatile ("out %0, r1" : : "M" (_SFR_IO_ADDR(BUF)));

				asm volatile ("out %0, r28" : : "M" (_SFR_IO_ADDR(APOS)));
			}
		}
		else if (LASTCMD == 0xf0) {

			if (((EEAR == 0) && (BYTE & POS))
					|| ((EEAR == 1) && !(BYTE & POS))) {
			//if (BYTE & POS) {
				DDRD = _BV(PD3);

				// 15us loop - r29 / r28 need not be preserved
				asm volatile ("ldi r29, 0");
				asm volatile ("ldi r28, 15"); // Z = 15
				asm volatile ("wdr"); // <-----
				asm volatile ("wdr");
				asm volatile ("wdr");
				asm volatile ("sbiw r28, 1");
				asm volatile ("cp r28, r1");
				asm volatile ("brne .-12"); // -^

				DDRD = 0;
				asm volatile ("wdr");
				EIFR |= _BV(INTF1);
			}
			if (EEAR < 2) {
				EEAR++;
			}
			else if (POS != 0x80) {
				POS <<= 1;
				EEAR = 0;
			}
			else {
				EEAR = 0;
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
					POS = 0;
				}
			}

		}
		asm volatile ("ldi r29, 0"); // LCNTH = 0
		asm volatile ("ldi r28, 1"); // LCNTL = 1

	}

}
