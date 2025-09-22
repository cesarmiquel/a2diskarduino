#include <avr/io.h>

//*****************************************************************************
// Function: StreamData
// Parameters: packet_buffer pointer
// Returns: always returns 0
//
// Description: This will stream the data in X until a 0x00 is found which
// indicates end of data. As input you get the packet_buffer address in
// r24 and r25. The output is sent to pin 6 of PORTD.
//
// This code is taken from:
//
// - https://gitlab.com/nyankat/smartportsd/-/blob/master/packet_16mhz.S
//
// I've removed the cycle count on each line but this code is cycle counted
// so you should not add a single NOP instruction which would screw all the
// timing. The code generates a 1us pulse and 3us delay (for a 1) and a 4us
// delay for a 0. It works perfectly with a 16Mhz Arduino Nano.
//
//*****************************************************************************
.global StreamData

StreamData:

          mov  XL,r24                 ;mov buffer pointer into X
          mov  XH,r25

          // set transmit bit low
          cbi  (_SFR_IO_ADDR(PORTD)),6

; Read a byte from the buffer and send it out on pin 6
nxtsbyte: ld   r23, x+                 ; get first byte from buffer
          cpi  r23, 0                  ; zero marks end of data
          breq enddata

          ldi  r25,8                   ; 8bits to read

; Process a bit
nxtsbit:  sbrs r23,7                   ; 1/2 send bit 7 first

          rjmp sbitclr                 ; bit is clear

; Send a 1us pulse and 3us delay
          sbi  (_SFR_IO_ADDR(PORTD)),6 ; set bit for 1us (14 cycles) --> 16 cycles for 16Mhz (2 more)

          ldi  r24,4                   ; delay total of 12 cycles
3:        dec  r24                     ; each loop +3 final loop +2
          brne 3b                      ;1/2 | 1 + 3x3 + 1x2 = 11

          nop
          nop

          cbi  (_SFR_IO_ADDR(PORTD)),6 ; clr bit for 3us (42 cycles) --> 48 cycles for 16Mhz (6 more)

          dec  r25                     ; dec bit counter
          breq nxtsbyt1
          rol  r23

          ldi  r24,13                  ; delay total of 39 cycles
3:        dec  r24
          brne 3b

          rjmp nxtsbit

nxtsbyt1:                              ; delay to makeup 3us (42 cycles total) --> 48 cycles for 16Mhz (6 more)

          ldi  r24,11                  ; delay total of 33 cycles
3:        dec  r24
          brne 3b

          nop

          rjmp nxtsbyte

; bit is clr, we need to check if its the last one, otherwise delay for 4us before next bit
sbitclr:  dec  r25
          breq nxtsbycl                ; 1/2 end of byte, delay then get nxt
          rol  r23
                                       ; delay to makeup 4us (56 cycles total) --> 64 cycles for 16Mhz (8 more)

          ldi  r24,18                  ; delay total of 54 cycles
3:        dec  r24
          brne 3b

          nop
          nop
          rjmp nxtsbit

; Finish sending a byte. A bit more delay and jump to process next byte
nxtsbycl:                              ; delay to makeup 4us (56 cycles total) --> 64 cycles for 16Mhz (8 more)

          ldi  r24,16                  ; delay total of 48 cycles
3:        dec  r24
          brne 3b

          nop
          nop
          nop

          rjmp nxtsbyte

enddata:
          clr  r25
          clr  r24                    ;return no error
          ret
