#include <avr/io.h>
#include <avr/interrupt.h>


; Onewire iButton / SmartButton slave.  Has the 64bit ID set below.
;
; Tested and working with a DS2482-100 and an IBL USB iButton reader. Should
; mostly adhere to the standard, but nothing is guaranteed.


; Set the 64bit ID (including 8bit CRC) here, in the order in which they are
; printed on the button (see
; <https://wiki.chaosdorf.de/images/f/fc/Smartbutton.jpg>).
; Note: The bytes are sent in reversed order. This has also been observed
; on off-the-shelf smartbuttons / iButtons.

#define ADDR1 0xC4
#define ADDR2 0x00
#define ADDR3 0x00
#define ADDR4 0x09
#define ADDR5 0x7d
#define ADDR6 0x79
#define ADDR7 0x04
#define ADDR8 0x01


; ==========================================================================
; You should not need to change things below, unless you're not using PD3
; as OWI data pin.
; ==========================================================================


; Register summary:
; r1:  NULLREG - constant 0 (set up by avr-gcc)
; r16: temporary
; r20: LASTCMD
; r21: BUF
; r22: POS
; r23: APOS
; r24: BYTE
; r25: SEARCHSTEP


; The LCNT registers count how many microseconds have passed
; since the last low-to-high / high-to-low transition. In the main loop,
; only r28 and r29 (Y) are incremented (precisely once per microsecond).
; Note that this requires an AVR operating at 8MHz.

#define LCNTH r29
#define LCNTL r28

; The last complete command byte received from the master. Once this is
; set, we start writing data onto the bus. Reset at bus resets and once a
; command is done
#define LASTCMD r20

; command buffer, always initialized to 0
#define BUF r21

; bitmask for the current buffer (either BUF or BYTE) position.
; Left-shifted after each bit
#define POS r22

; Position in the 8-byte address sequence
#define APOS r23

; current byte in this sequence
#define BYTE r24

; the SEARCH ROM command consits of three steps: send a bit of our address,
; send the inverted bit of our address, receive the bit the master chose
; to proceed with. the current position in this cycle is stored here
#define SEARCHSTEP r25

#define CMD_READROM 0x33
#define CMD_SEARCHROM 0xf0

#define SEARCHSTEP_BIT 0
#define SEARCHSTEP_INV 1
#define SEARCHSTEP_DIRECTION 2

#define NULLREG r1

.text

.global main

main:
	; watchdog reset after ~4 seconds
	out _SFR_IO_ADDR(MCUSR), NULLREG
	ldi r16, (_BV(WDCE) | _BV(WDE))
	out _SFR_IO_ADDR(WDTCR), r16
	ldi r16, (_BV(WDE) | _BV(WDP3))
	out _SFR_IO_ADDR(WDTCR), r16

	; rising edge for reset/presence signals and reading data,
	; falling edge for writing.
	ldi r16, _BV(ISC10)
	out _SFR_IO_ADDR(MCUCR), r16
	ldi r16, _BV(INT1)
	out _SFR_IO_ADDR(GIMSK), r16

	; disable Analog Comparator
	sbi _SFR_IO_ADDR(ACSR), ACD

	; disable USI / USART
	sbi _SFR_IO_ADDR(PRR), PRUSI
	sbi _SFR_IO_ADDR(PRR), PRUSART

	clr POS
	clr APOS
	clr _SFR_IO_ADDR(DDRD)
	clr _SFR_IO_ADDR(PORTD)
	sbi _SFR_IO_ADDR(DDRB), PB2
	cbi _SFR_IO_ADDR(PORTB), PB2

	sei

	clr LCNTL
	clr LCNTH
	; increment LCNT once per microsecond
loop:
	adiw LCNTL, 1
	wdr
	nop
	nop
	nop
	nop
	rjmp loop

delay_short:
	ldi r16, 15
	wdr
	wdr
	wdr
	wdr
	subi r16, 1
	cpi r16, 0
	brne .-14
	ret

delay_long:
	ldi r16, 120
	wdr
	wdr
	wdr
	wdr
	subi r16, 1
	cpi r16, 0
	brne .-14
	ret

.global INT1_vect

INT1_vect:
	sbis _SFR_IO_ADDR(PIND), PD3
	rjmp check_cmd

	; Read OWI command
	cpi LCNTH, 0
	breq check_lastcmd

	; send presence signal
	sbi _SFR_IO_ADDR(DDRD), PD3
	rcall delay_long
	cbi _SFR_IO_ADDR(DDRD), PD3
	clr LASTCMD
	clr BUF
	ldi POS, 1
	clr APOS
	wdr
	in r16, _SFR_IO_ADDR(GIFR)
	ori r16, _BV(INTF1)
	out _SFR_IO_ADDR(GIFR), r16
	reti

check_lastcmd:
	cpi LASTCMD, 0
	brne check_lastcmd_nope
	cpi LCNTL, 16
	brsh check_lastcmd_lcntl_done
	or BUF, POS
check_lastcmd_lcntl_done:
	cpi POS, 0x80
	breq command_received
	lsl POS
check_lastcmd_nope:
	reti

command_received:
	mov LASTCMD, BUF
	ldi POS, 1
	clr APOS
	ldi BYTE, ~ADDR8
	ldi SEARCHSTEP, SEARCHSTEP_BIT
	in r16, _SFR_IO_ADDR(GIFR)
	ori r16, _BV(INTF1)
	out _SFR_IO_ADDR(GIFR), r16
	reti

check_cmd:
	cpi LASTCMD, CMD_READROM
	brne check_cmd_searchrom

	; we got READ ROM
	mov r16, BYTE
	and r16, POS
	breq pos_bit_is_null

	; (BYTE & POS) is true -> ADDRx has a 0 bit, keep data low
	sbi _SFR_IO_ADDR(DDRD), PD3
	rcall delay_short
	cbi _SFR_IO_ADDR(DDRD), PD3
	wdr
	in r16, _SFR_IO_ADDR(GIFR)
	ori r16, _BV(INTF1)
	out _SFR_IO_ADDR(GIFR), r16

pos_bit_is_null:
	cpi POS, 0x80
	breq pos_is_0x80
	lsl POS
	rjmp check_cmd_cleanup

pos_is_0x80:
	ldi POS, 1
	; (1)
	; Put next ADDRx into BYTE or reset state if we sent all 8 bytes.
	; APOS is checked for 1 / 2 / 3 /... in turn and the corresponding
	; address set where appropriate. Since there are no shortcuts, this
	; block has a constant execution time of ~4us
	inc APOS         ;APOS++
	cpi APOS, 1
	brne .+2        ; if (APOS == 1) {
	ldi BYTE, ~ADDR7
	cpi APOS, 2      ; }
	brne .+2        ; if (APOS == 2) {
	ldi BYTE, ~ADDR6
	cpi APOS, 3      ; }
	brne .+2        ; if (APOS == 3) {
	ldi BYTE, ~ADDR5
	cpi APOS, 4      ; }
	brne .+2        ; if (APOS == 4) {
	ldi BYTE, ~ADDR4
	cpi APOS, 5      ; }
	brne .+2        ; if (APOS == 5) {
	ldi BYTE, ~ADDR3
	cpi APOS, 6      ; }
	brne .+2        ; if (APOS == 6) {
	ldi BYTE, ~ADDR2
	cpi APOS, 7      ; }
	brne .+2        ; if (APOS == 7) {
	ldi BYTE, ~ADDR1
	cpi APOS, 8     ; }
	brne .+4       ; if (APOS == 8) {
	clr LASTCMD
	clr BUF        ; }
	rjmp check_cmd_cleanup


check_cmd_searchrom:
	cpi LASTCMD, CMD_SEARCHROM
	brne check_cmd_cleanup

	cpi SEARCHSTEP, SEARCHSTEP_BIT
	brne check_searchstep_2
	mov r16, BYTE
	and r16, POS
	brne check_searchstep_2
	rjmp send_ack

check_searchstep_2:
	cpi SEARCHSTEP, SEARCHSTEP_INV
	brne check_search_lt_direction
	mov r16, BYTE
	and r16, POS
	breq check_search_lt_direction

send_ack:
	rcall delay_short
	cbi _SFR_IO_ADDR(DDRD), PD3
	wdr
	in r16, _SFR_IO_ADDR(GIFR)
	ori r16, _BV(INTF1)
	out _SFR_IO_ADDR(GIFR), r16

check_search_lt_direction:
	cpi SEARCHSTEP, SEARCHSTEP_DIRECTION
	brge check_pos
	inc SEARCHSTEP
check_pos:
	cpi POS, 0x80
	breq check_pos_else
	lsl POS
	ldi SEARCHSTEP, SEARCHSTEP_BIT
	rjmp check_cmd_cleanup
check_pos_else:
	ldi SEARCHSTEP, SEARCHSTEP_BIT
	ldi POS, 1

	; see (1)
	inc APOS
	cpi APOS, 1
	brne .+2
	ldi BYTE, ~ADDR7
	cpi APOS, 2
	brne .+2
	ldi BYTE, ~ADDR6
	cpi APOS, 3
	brne .+2
	ldi BYTE, ~ADDR5
	cpi APOS, 4
	brne .+2
	ldi BYTE, ~ADDR4
	cpi APOS, 5
	brne .+2
	ldi BYTE, ~ADDR3
	cpi APOS, 6
	brne .+2
	ldi BYTE, ~ADDR2
	cpi APOS, 7
	brne .+2
	ldi BYTE, ~ADDR1
	cpi APOS, 8
	brne .+4
	clr LASTCMD
	clr BUF


check_cmd_cleanup:
	clr LCNTH
	ldi LCNTL, 1
	reti
