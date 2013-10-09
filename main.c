#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

/*
 * Onewire iButton / SmartButton slave.  Has the 64bit ID set below.
 *
 * Only supports non-overdrive READ ROM. Does not hold any data.
 * SEARCH ROM Support is work in progress. Command byte readout is pretty
 * broken right now.
 *
 * Tested and working with a DS2482. Should mostly adhere to the standard,
 * but nothing is guaranteed.
 */



/*
 * Set the 64bit ID (including 8bit CRC) here, in the order in which they are
 * printed on the button (see
 * <https://wiki.chaosdorf.de/images/f/fc/Smartbutton.jpg>).
 * Note: The bytes are sent in reversed order. This has also been observed
 * on off-the-shelf smartbuttons / iButtons.
 */
#define ADDR1 0xC4
#define ADDR2 0x00
#define ADDR3 0x00
#define ADDR4 0x09
#define ADDR5 0x7d
#define ADDR6 0x79
#define ADDR7 0x04
#define ADDR8 0x01


/*
 * You should not need to change things below, unless you're not using PD3
 * as OWI data pin.
 */




/*
 * RAM access is time-expensive and requires the X / Y / Z registers. Since
 * we have neither time nor many available registers (Y and Z are used
 * otherwise during the main loop), all program variables are saved in
 * registers.
 *
 * The LCNT registers count how many microseconds have passed
 * since the last low-to-high / high-to-low transition. In the main loop,
 * only r28 and r29 (Y) are incremented (precisely once per microsecond),
 * their GPIO counterparts are used in the ISR and synced with the registers
 * both at start and end.
 *
 * Note: The compiler knows that these registers are forbiden thanks to
 * -ffixed-28 -ffixed-29
 *
 * LCNT = r28 (YL), r29 (YH)
 */


#define LCNTH GPIOR2
#define LCNTL GPIOR1

#define CNT USIDR
#define LASTCMD OCR0B
#define BUF OCR0A
#define POS OCR1A
#define APOS OCR1B
#define BYTE EEDR


#define delay_us_Y(delay) \
	asm volatile ("ldi r29, 0"); \
	asm volatile ("ldi r28, %0" : : "M" (delay)); \
	asm volatile ("wdr"); \
	asm volatile ("wdr"); \
	asm volatile ("wdr"); \
	asm volatile ("sbiw r28, 1"); \
	asm volatile ("cp r28, r1"); \
	asm volatile ("brne .-12");

int main (void)
{
	/* watchdog reset after ~4 seconds - just in case */
	MCUSR &= ~_BV(WDRF);
	WDTCR = _BV(WDCE) | _BV(WDE);
	WDTCR = _BV(WDE) | _BV(WDP3);

	/*
	 * rising edge for reset/presence signals and reading data,
	 * falling edge for writing.
	 */
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

	/*
	 * takes exactly 1us per cycle
	 */
loop:
	asm volatile ("adiw r28, 1"); // 2c
	asm volatile ("wdr"); // 1c
	asm volatile ("nop"); // 1c
	asm volatile ("nop"); // 1c
	asm volatile ("nop"); // 1c
	asm volatile ("nop"); // 1c
	goto loop; // 1c

	return 0;
}

ISR(INT1_vect)
{
	// overhead: 19c (2.4us)
	if (PIND & _BV(PD3)) {

		/*
		 * Make LCNT available to the C compiler
		 */
		asm volatile ("out %0, r29" : : "M" (_SFR_IO_ADDR(LCNTH)));
		asm volatile ("out %0, r28" : : "M" (_SFR_IO_ADDR(LCNTL)));

		/*
		 * Line was high for >256us - got reset signal, send presence
		 */
		if (LCNTH > 0) {
			DDRD = _BV(PD3);

			/*
			 * Y register is no longer needed at this point and may be
			 * overwritten.
			 */
			delay_us_Y(120);

			DDRD = 0;
			LASTCMD = 0;
			BUF = 0;
			POS = 1;
			APOS = 0;
			asm volatile ("wdr");
			EIFR |= _BV(INTF1);
		}
		else if (!LASTCMD) {
			/*
			* Line was high for > 15us - got a "write 0"
			*/
			if (LCNTL > 15) {
				// nothing to do
			}
			/*
			* Line was high for <= 15us - got a "write 1". Might also be a
			* "read", so only do stuff if we don't have a command set
			*/
			else {
				BUF |= POS;
			}
			/*
			* We received 8 command bits. Store the command and switch to
			* write mode (also, store the first byte to be sent)
			*/
			if (POS != 0x80) {
				POS <<= 1;
			}
			else {
				LASTCMD = BUF;
				POS = 1;
				APOS = 0;
				BYTE = ~ADDR8;
				EEAR = 0;
				EIFR |= _BV(INTF1);
			}
		}
	}
	else {
		if (LASTCMD == 0x33) {

			/*
			 * ~ADDRx & POS == 1 -> ADDRx has a 0 bit which is sent by
			 * keeping the data line low after the master released it.
			 */
			if (BYTE & POS) {

				DDRD = _BV(PD3);

				delay_us_Y(15);

				DDRD = 0;
				asm volatile ("wdr");
				EIFR |= _BV(INTF1);
			}
			if (POS != 0x80) {
				POS <<= 1;
			}
			else {
				POS = 1;
				/*
				 * Put next ADDRx into BYTE or reset state if we sent all 8
				 * bytes. Again, a RAM array is too expensive, and both
				 * if/elseif and case/when chains are expensive too. What
				 * happens here is the following:
				 *
				 * APOS is stored in r28, BYTE in r29 (both are written
				 * back at the end of the block). Then APOS is checked for
				 * 1 / 2 / 3 /... in turn and the corresponding address set
				 * where appropriate. Since there are no shortcuts, this
				 * block has a constant execution time of 4us
				 * (compared to ~10us with an if/else if chain and -Os)
				 */
				asm volatile ("in r28, %0" : : "M" (_SFR_IO_ADDR(APOS)));
				asm volatile ("inc r28");     // APOS++
				asm volatile ("cpi r28, 1");
				asm volatile ("brne .+2");   // if (APOS == 1) {
				asm volatile ("ldi r29, %0" : : "i" (~ADDR7));
				asm volatile ("cpi r28, 2"); // }
				asm volatile ("brne .+2");   // else if (APOS == 2) {
				asm volatile ("ldi r29, %0" : : "i" (~ADDR6));
				asm volatile ("cpi r28, 3"); // }
				asm volatile ("brne .+2");   // else if (APOS == 3) {
				asm volatile ("ldi r29, %0" : : "i" (~ADDR5));
				asm volatile ("cpi r28, 4"); // }
				asm volatile ("brne .+2");   // else if (APOS == 4) {
				asm volatile ("ldi r29, %0" : : "i" (~ADDR4));
				asm volatile ("cpi r28, 5"); // }
				asm volatile ("brne .+2");   // else if (APOS == 5) {
				asm volatile ("ldi r29, %0" : : "i" (~ADDR3));
				asm volatile ("cpi r28, 6"); // }
				asm volatile ("brne .+2");   // else if (APOS == 6) {
				asm volatile ("ldi r29, %0" : : "i" (~ADDR2));
				asm volatile ("cpi r28, 7"); // }
				asm volatile ("brne .+2");   // else if (APOS == 7) {
				asm volatile ("ldi r29, %0" : : "i" (~ADDR1));
				                             // }

				asm volatile ("cpi r28, 8");
				asm volatile ("brne .+4");   // if (APOS == 8) {
				asm volatile ("out %0, r1" : : "M" (_SFR_IO_ADDR(LASTCMD)));
				asm volatile ("out %0, r1" : : "M" (_SFR_IO_ADDR(BUF)));
				                             // }

				asm volatile ("out %0, r28" : : "M" (_SFR_IO_ADDR(APOS)));
				asm volatile ("out %0, r29" : : "M" (_SFR_IO_ADDR(BYTE)));
			}
		}
		else if (LASTCMD == 0xf0) {

			if (((EEAR == 0) && (BYTE & POS))
					|| ((EEAR == 1) && !(BYTE & POS))) {
			//if (BYTE & POS) {
				DDRD = _BV(PD3);

				delay_us_Y(15);

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
			}

		}
		asm volatile ("ldi r29, 0"); // LCNTH = 0
		asm volatile ("ldi r28, 1"); // LCNTL = 1

	}

}
