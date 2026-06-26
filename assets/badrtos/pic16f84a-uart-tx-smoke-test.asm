; ------------------------------------------------------------
; PIC16F84A software UART TX smoke test
;
; Project:
;   PIC16F84A TinyRTOS / Shell bring-up
;
; Purpose:
;   First firmware step before building the cooperative runtime.
;   Sends a repeating UART message so the PIC can prove it is alive
;   using only bit-banged serial.
;
; Target:
;   PIC16F84A
;
; Clock:
;   External clock into OSC1/CLKIN
;   Intended Fosc = 1 MHz from AD3 DIO3
;
; UART:
;   TX on RB0 / pin 6
;   4800 baud-ish
;   8 data bits
;   no parity
;   1 stop bit
;   idle high
;
; Message:
;   "PIC16F84A alive\r\n"
;
; Wiring:
;   PIC pin 14 VDD        -> +5 V
;   PIC pin 5  VSS        -> GND
;   PIC pin 16 OSC1/CLKIN -> AD3 DIO3 through ~1k
;   PIC pin 6  RB0/TX     -> serial RX
;
; If serial RX is 3.3 V only:
;
;   PIC RB0 ---- 10k ----+---- RX
;                        |
;                       20k
;                        |
;                       GND
;
; Notes:
;   This source is for documentation/recordkeeping.
;   The working HEX was generated directly using a Python PIC14 emitter,
;   because installing/building gputils/gpasm on Windows was annoying.
; ------------------------------------------------------------

        LIST    P=16F84A
        INCLUDE <p16f84a.inc>

; ------------------------------------------------------------
; RAM variables
; PIC16F84A general purpose RAM starts at 0x0C.
; ------------------------------------------------------------

        CBLOCK  0x0C
        TXBYTE
        BITCNT
        DLY
        GAP1
        GAP2
        ENDC

; ------------------------------------------------------------
; Constants
; ------------------------------------------------------------

TXBIT   EQU     0           ; RB0

; ------------------------------------------------------------
; Reset vector
; ------------------------------------------------------------

        ORG     0x0000
        GOTO    start

; ------------------------------------------------------------
; Init
; ------------------------------------------------------------

start:
        ; RB0 output, RB1-RB7 inputs.
        ; Legacy TRIS instruction used because this is the cursed old PIC way.
        MOVLW   b'11111110'
        TRIS    PORTB

        ; UART idle state is high.
        BSF     PORTB, TXBIT

main_loop:
        CALL    send_message
        CALL    long_gap
        GOTO    main_loop

; ------------------------------------------------------------
; send_message
;
; Sends:
;   PIC16F84A alive\r\n
; ------------------------------------------------------------

send_message:
        MOVLW   'P'
        CALL    uart_tx

        MOVLW   'I'
        CALL    uart_tx

        MOVLW   'C'
        CALL    uart_tx

        MOVLW   '1'
        CALL    uart_tx

        MOVLW   '6'
        CALL    uart_tx

        MOVLW   'F'
        CALL    uart_tx

        MOVLW   '8'
        CALL    uart_tx

        MOVLW   '4'
        CALL    uart_tx

        MOVLW   'A'
        CALL    uart_tx

        MOVLW   ' '
        CALL    uart_tx

        MOVLW   'a'
        CALL    uart_tx

        MOVLW   'l'
        CALL    uart_tx

        MOVLW   'i'
        CALL    uart_tx

        MOVLW   'v'
        CALL    uart_tx

        MOVLW   'e'
        CALL    uart_tx

        MOVLW   0x0D        ; carriage return
        CALL    uart_tx

        MOVLW   0x0A        ; newline
        CALL    uart_tx

        RETURN

; ------------------------------------------------------------
; uart_tx
;
; Input:
;   W = byte to transmit
;
; Sends:
;   start bit = 0
;   8 data bits, LSB first
;   stop bit = 1
;
; Timing:
;   Intended for Fosc = 1 MHz.
;   PIC instruction clock = Fosc / 4 = 250 kHz.
;   One instruction cycle = 4 us.
;   4800 baud bit time = about 208 us.
;
; This is approximate and good enough for first bring-up.
; ------------------------------------------------------------

uart_tx:
        MOVWF   TXBYTE

        ; Start bit.
        BCF     PORTB, TXBIT
        CALL    bit_delay

        ; 8 data bits.
        MOVLW   .8
        MOVWF   BITCNT

tx_bit_loop:
        ; Test LSB of TXBYTE.
        ; If set, send one. Otherwise send zero.
        BTFSC   TXBYTE, 0
        GOTO    tx_send_one

tx_send_zero:
        BCF     PORTB, TXBIT
        GOTO    tx_after_bit_set

tx_send_one:
        BSF     PORTB, TXBIT

tx_after_bit_set:
        CALL    bit_delay

        ; Shift next bit into position.
        BCF     STATUS, C
        RRF     TXBYTE, F

        DECFSZ  BITCNT, F
        GOTO    tx_bit_loop

        ; Stop bit.
        BSF     PORTB, TXBIT
        CALL    bit_delay

        RETURN

; ------------------------------------------------------------
; bit_delay
;
; Rough delay for 4800 baud at Fosc = 1 MHz.
; ------------------------------------------------------------

bit_delay:
        MOVLW   .15
        MOVWF   DLY

bit_delay_loop:
        DECFSZ  DLY, F
        GOTO    bit_delay_loop

        RETURN

; ------------------------------------------------------------
; long_gap
;
; Delay between repeated messages so the serial output is readable.
; ------------------------------------------------------------

long_gap:
        MOVLW   .80
        MOVWF   GAP1

gap_outer:
        MOVLW   .255
        MOVWF   GAP2

gap_inner:
        DECFSZ  GAP2, F
        GOTO    gap_inner

        DECFSZ  GAP1, F
        GOTO    gap_outer

        RETURN

        END
