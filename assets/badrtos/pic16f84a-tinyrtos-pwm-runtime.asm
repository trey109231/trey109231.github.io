; ------------------------------------------------------------
;TinyRTOS-ish Bring-up
;
; Features:
;   - Timer0 interrupt tick
;   - 16-bit uptime tick counter
;   - countdown-based cooperative task timing
;   - heartbeat LED on RB1, active-low
;   - debounced button input on RB2, active-high
;   - button press counter
;   - button cycles heartbeat mode 0..2
;   - button cycles PWM mode/duty 0..4
;   - 4-phase software PWM on RB3, active-low
;   - bit-banged UART TX on RB0, 4800 baud-ish, 8N1, idle-high
;   - boot message and periodic telemetry line
;
; Runtime wiring:
;   PIC pin 14 VDD        -> +5 V
;   PIC pin 5  VSS        -> GND
;   PIC pin 16 OSC1/CLKIN -> AD3 DIO3 clock through ~1k
;   PIC pin 6  RB0        -> UART TX, idle-high
;   PIC pin 7  RB1        -> heartbeat LED, active-low
;   PIC pin 8  RB2        -> button input, active-high
;   PIC pin 9  RB3        -> PWM LED, active-low
;
; Button wiring:
;   +5 V -> button -> RB2
;   RB2 -> 10k -> GND
;
; LED wiring, active-low:
;   +5 V -> resistor -> LED anode
;   LED cathode -> PIC output pin
;
; Clock assumptions:
;   External Fosc = 1 MHz from AD3 DIO3.
;   Instruction clock = Fosc / 4 = 250 kHz.
;   OPTION_REG = 0x81 gives Timer0 internal clock with 1:4 prescaler.
;   Timer0 overflow period ~= 256 * 4 * 4 us = 4.096 ms.
;
; Programmer flow:
;   python pic16f84a_ad3_programmer.py --flash-hex pwm_runtime_4800_pic16f84a.hex --yes
;   python pic16f84a_ad3_programmer.py --run-clock --clock-hz 1000000
;
; Serial monitor:
;   4800 baud, 8N1
; ------------------------------------------------------------

        LIST    P=16F84A
        INCLUDE <p16f84a.inc>

; ------------------------------------------------------------
; Pin aliases
; ------------------------------------------------------------

TX_BIT          EQU     0       ; RB0 UART TX, idle-high
HEARTBEAT_LED   EQU     1       ; RB1 heartbeat LED, active-low
BUTTON_BIT      EQU     2       ; RB2 button input, active-high
PWM_LED         EQU     3       ; RB3 PWM LED, active-low

; ------------------------------------------------------------
; Bit aliases
; ------------------------------------------------------------

C_BIT           EQU     0       ; STATUS.C
Z_BIT           EQU     2       ; STATUS.Z
RP0_BIT         EQU     5       ; STATUS.RP0

T0IF_BIT        EQU     2       ; INTCON.T0IF
T0IE_BIT        EQU     5       ; INTCON.T0IE
GIE_BIT         EQU     7       ; INTCON.GIE

BTN_STABLE_BIT  EQU     0       ; BUTTON_STATE bit 0: debounced pressed state

; ------------------------------------------------------------
; PIC16F84A general-purpose RAM starts at 0x0C.
; ------------------------------------------------------------

        CBLOCK  0x0C
TICK_LO             ; 0x0C 16-bit system tick, low byte
TICK_HI             ; 0x0D 16-bit system tick, high byte
W_TEMP              ; 0x0E ISR W save
STATUS_TEMP         ; 0x0F ISR STATUS save
HB_COUNTDOWN        ; 0x10 heartbeat countdown, decremented in ISR
STATUS_COUNTDOWN    ; 0x11 status print countdown, decremented in ISR
TXBYTE              ; 0x12 UART transmit byte
BITCNT              ; 0x13 UART bit counter
BAUD_DLY            ; 0x14 UART bit delay counter
SNAP_TICK_LO        ; 0x15 status snapshot tick low
SNAP_TICK_HI        ; 0x16 status snapshot tick high
HEX_TMP             ; 0x17 print_hex8 temporary byte
BTN_POLL_COUNTDOWN  ; 0x18 button task countdown, decremented in ISR
DEBOUNCE_COUNT      ; 0x19 debounce integrator/count
BUTTON_STATE        ; 0x1A bit0 = debounced pressed state
PRESS_LO            ; 0x1B 16-bit accepted press count, low byte
PRESS_HI            ; 0x1C 16-bit accepted press count, high byte
HEARTBEAT_MODE      ; 0x1D heartbeat mode 0..2
SNAP_PRESS_LO       ; 0x1E status snapshot press low
SNAP_PRESS_HI       ; 0x1F status snapshot press high
NIBBLE_TMP          ; 0x20 hex nibble temporary
PWM_PHASE           ; 0x21 PWM phase 0..3
PWM_MODE            ; 0x22 PWM mode/duty 0..4
        ENDC

; ------------------------------------------------------------
; Vectors
; ------------------------------------------------------------

        ORG     0x0000
        GOTO    start


        NOP
        NOP
        NOP

        ORG     0x0004

; ------------------------------------------------------------
; Timer0 ISR
;
; Responsibilities:
;   - save W and STATUS
;   - handle Timer0 overflow
;   - increment 16-bit tick counter
;   - decrement task countdowns when nonzero
;   - update 4-phase software PWM on RB3
; ------------------------------------------------------------

isr:
        MOVWF   W_TEMP
        SWAPF   STATUS, W
        MOVWF   STATUS_TEMP

        ; Force bank 0 for SFR and RAM access.
        BCF     STATUS, RP0_BIT

        ; Only handle Timer0 interrupt if T0IF is set.
        BTFSS   INTCON, T0IF_BIT
        GOTO    isr_restore

        ; Clear Timer0 interrupt flag.
        BCF     INTCON, T0IF_BIT

        ; 16-bit tick++.
        INCF    TICK_LO, F
        BTFSC   STATUS, Z_BIT
        INCF    TICK_HI, F

        ; if HB_COUNTDOWN != 0: HB_COUNTDOWN--
        MOVF    HB_COUNTDOWN, F
        BTFSC   STATUS, Z_BIT
        GOTO    isr_hb_done
        DECF    HB_COUNTDOWN, F

isr_hb_done:
        ; if STATUS_COUNTDOWN != 0: STATUS_COUNTDOWN--
        MOVF    STATUS_COUNTDOWN, F
        BTFSC   STATUS, Z_BIT
        GOTO    isr_status_done
        DECF    STATUS_COUNTDOWN, F

isr_status_done:
        ; if BTN_POLL_COUNTDOWN != 0: BTN_POLL_COUNTDOWN--
        MOVF    BTN_POLL_COUNTDOWN, F
        BTFSC   STATUS, Z_BIT
        GOTO    isr_button_done
        DECF    BTN_POLL_COUNTDOWN, F

isr_button_done:
        ; PWM_PHASE = (PWM_PHASE + 1) modulo 4.
        INCF    PWM_PHASE, F
        MOVLW   .4
        SUBWF   PWM_PHASE, W
        BTFSC   STATUS, Z_BIT
        CLRF    PWM_PHASE

        ; If PWM_MODE == 0, PWM always off.
        MOVF    PWM_MODE, F
        BTFSC   STATUS, Z_BIT
        GOTO    pwm_off

        ; If PWM_MODE == 4, PWM always on.
        MOVLW   .4
        SUBWF   PWM_MODE, W
        BTFSC   STATUS, Z_BIT
        GOTO    pwm_on

        ; Otherwise compare PWM_PHASE with PWM_MODE.
        ; This gives a tiny 4-phase PWM:
        ;   mode 1 -> about 1/4
        ;   mode 2 -> about 2/4
        ;   mode 3 -> about 3/4
        ;   mode 4 -> always on
        MOVF    PWM_PHASE, W
        SUBWF   PWM_MODE, W
        BTFSC   STATUS, Z_BIT
        GOTO    pwm_off
        BTFSS   STATUS, C_BIT
        GOTO    pwm_off

pwm_on:
        ; Active-low LED on RB3.
        BCF     PORTB, PWM_LED
        GOTO    isr_restore

pwm_off:
        ; Active-low LED off on RB3.
        BSF     PORTB, PWM_LED

isr_restore:
        SWAPF   STATUS_TEMP, W
        MOVWF   STATUS
        SWAPF   W_TEMP, F
        SWAPF   W_TEMP, W
        RETFIE

; ------------------------------------------------------------
; Initialization
; ------------------------------------------------------------

start:
        ; Bank 0.
        BCF     STATUS, RP0_BIT

        ; Initial output latch:
        ;   RB0 = 1 UART idle high
        ;   RB1 = 1 heartbeat LED off, active-low
        ;   RB2 = 0 input latch low
        ;   RB3 = 1 PWM LED off, active-low
        MOVLW   b'00001011'
        MOVWF   PORTB

        ; TRISB:
        ;   RB0 output UART TX
        ;   RB1 output heartbeat LED
        ;   RB2 input button
        ;   RB3 output PWM LED
        ;   RB4-RB7 inputs
        MOVLW   b'11110100'
        TRIS    PORTB

        ; Clear runtime state.
        CLRF    TICK_LO
        CLRF    TICK_HI
        CLRF    DEBOUNCE_COUNT
        CLRF    BUTTON_STATE
        CLRF    PRESS_LO
        CLRF    PRESS_HI
        CLRF    HEARTBEAT_MODE
        CLRF    PWM_PHASE
        CLRF    PWM_MODE

        ; Load initial heartbeat countdown based on HEARTBEAT_MODE.
        CALL    load_heartbeat_reload

        ; Initial status print countdown = 0xF4.
        MOVLW   0xF4
        MOVWF   STATUS_COUNTDOWN

        ; Button polling countdown = 4 ticks.
        MOVLW   .4
        MOVWF   BTN_POLL_COUNTDOWN

        ; OPTION_REG = 0x81 while bank 1 is selected.
        ;   RBPU  = 1, PORTB pullups disabled
        ;   INTEDG= 0
        ;   T0CS  = 0, Timer0 internal instruction clock
        ;   T0SE  = 0
        ;   PSA   = 0, prescaler assigned to Timer0
        ;   PS2:0 = 001, Timer0 prescaler 1:4
        BSF     STATUS, RP0_BIT
        MOVLW   b'10000001'
        MOVWF   OPTION_REG
        BCF     STATUS, RP0_BIT

        ; Clear Timer0 interrupt flag and enable Timer0 interrupts.
        BCF     INTCON, T0IF_BIT
        BSF     INTCON, T0IE_BIT
        BSF     INTCON, GIE_BIT

        ; Boot message.
        CALL    send_boot_message

; ------------------------------------------------------------
; Main cooperative loop
; ------------------------------------------------------------

main_loop:
        CALL    task_heartbeat
        CALL    task_button
        CALL    task_status
        GOTO    main_loop

; ------------------------------------------------------------
; load_heartbeat_reload
;
; HEARTBEAT_MODE controls heartbeat delay:
;   mode 0 -> 0x7A ~= 122 Timer0 overflows ~= 0.5 s at 1 MHz
;   mode 1 -> 0x28 ~=  40 Timer0 overflows ~= 0.16 s at 1 MHz
;   mode 2 -> 0xF4 ~= 244 Timer0 overflows ~= 1.0 s at 1 MHz
; ------------------------------------------------------------

load_heartbeat_reload:
        MOVF    HEARTBEAT_MODE, W
        BTFSC   STATUS, Z_BIT
        GOTO    hb_mode_0

        MOVLW   .1
        SUBWF   HEARTBEAT_MODE, W
        BTFSC   STATUS, Z_BIT
        GOTO    hb_mode_1

        ; Else mode 2.
hb_mode_2:
        MOVLW   0xF4
        MOVWF   HB_COUNTDOWN
        RETURN

hb_mode_1:
        MOVLW   0x28
        MOVWF   HB_COUNTDOWN
        RETURN

hb_mode_0:
        MOVLW   0x7A
        MOVWF   HB_COUNTDOWN
        RETURN

; ------------------------------------------------------------
; task_heartbeat
;
; When HB_COUNTDOWN reaches zero, reload based on current mode and
; toggle RB1. RB1 LED is active-low.
; ------------------------------------------------------------

task_heartbeat:
        MOVF    HB_COUNTDOWN, F
        BTFSS   STATUS, Z_BIT
        GOTO    heartbeat_return

        CALL    load_heartbeat_reload

        ; Toggle active-low heartbeat LED on RB1.
        BTFSC   PORTB, HEARTBEAT_LED
        GOTO    heartbeat_turn_on

heartbeat_turn_off:
        BSF     PORTB, HEARTBEAT_LED
        GOTO    heartbeat_return

heartbeat_turn_on:
        BCF     PORTB, HEARTBEAT_LED

heartbeat_return:
        RETURN

; ------------------------------------------------------------
; task_button
;
; Runs only when BTN_POLL_COUNTDOWN reaches zero. It then reloads the
; countdown to 4 ticks. With ~4.096 ms Timer0 ticks, this polls about
; every 16 ms.
;
; Debounce threshold is 3 consecutive differing polls. An accepted rising
; edge increments the press counter and cycles both heartbeat and PWM modes.
; ------------------------------------------------------------

task_button:
        MOVF    BTN_POLL_COUNTDOWN, F
        BTFSS   STATUS, Z_BIT
        GOTO    button_return

        MOVLW   .4
        MOVWF   BTN_POLL_COUNTDOWN

        ; Read raw button on RB2.
        ; If raw high, go to raw_high. Otherwise raw low path.
        BTFSC   PORTB, BUTTON_BIT
        GOTO    button_raw_high

button_raw_low:
        ; If stable state is already low, no change: clear debounce count.
        BTFSS   BUTTON_STATE, BTN_STABLE_BIT
        GOTO    button_same_low
        GOTO    button_diff_low

button_same_low:
        CLRF    DEBOUNCE_COUNT
        GOTO    button_return

button_diff_low:
        ; Raw low while stable high: possible release.
        INCF    DEBOUNCE_COUNT, F
        MOVLW   .3
        SUBWF   DEBOUNCE_COUNT, W
        BTFSS   STATUS, Z_BIT
        GOTO    button_return

        ; Accept stable release.
        CLRF    DEBOUNCE_COUNT
        BCF     BUTTON_STATE, BTN_STABLE_BIT
        GOTO    button_return

button_raw_high:
        ; If stable state is already high, no change: clear debounce count.
        BTFSC   BUTTON_STATE, BTN_STABLE_BIT
        GOTO    button_same_high
        GOTO    button_diff_high

button_same_high:
        CLRF    DEBOUNCE_COUNT
        GOTO    button_return

button_diff_high:
        ; Raw high while stable low: possible press.
        INCF    DEBOUNCE_COUNT, F
        MOVLW   .3
        SUBWF   DEBOUNCE_COUNT, W
        BTFSS   STATUS, Z_BIT
        GOTO    button_return

        ; Accept stable press.
        CLRF    DEBOUNCE_COUNT
        BSF     BUTTON_STATE, BTN_STABLE_BIT

        ; press_count++
        INCF    PRESS_LO, F
        BTFSC   STATUS, Z_BIT
        INCF    PRESS_HI, F

        ; HEARTBEAT_MODE = (HEARTBEAT_MODE + 1) modulo 3.
        INCF    HEARTBEAT_MODE, F
        MOVLW   .3
        SUBWF   HEARTBEAT_MODE, W
        BTFSC   STATUS, Z_BIT
        CLRF    HEARTBEAT_MODE

        ; PWM_MODE = (PWM_MODE + 1) modulo 5.
        INCF    PWM_MODE, F
        MOVLW   .5
        SUBWF   PWM_MODE, W
        BTFSC   STATUS, Z_BIT
        CLRF    PWM_MODE

        ; Reload heartbeat immediately for the new mode.
        CALL    load_heartbeat_reload

button_return:
        RETURN

; ------------------------------------------------------------
; task_status
;
; When STATUS_COUNTDOWN reaches zero, reload it and print one telemetry line.
; ------------------------------------------------------------

task_status:
        MOVF    STATUS_COUNTDOWN, F
        BTFSS   STATUS, Z_BIT
        GOTO    status_return

        MOVLW   0xF4
        MOVWF   STATUS_COUNTDOWN
        CALL    send_status_line

status_return:
        RETURN

; ------------------------------------------------------------
; send_boot_message
;
; Sends:
;   CR LF
;   PIC16F84A pwm runtime
;   CR LF
; ------------------------------------------------------------

send_boot_message:
        MOVLW   0x0D
        CALL    uart_tx
        MOVLW   0x0A
        CALL    uart_tx

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
        MOVLW   'p'
        CALL    uart_tx
        MOVLW   'w'
        CALL    uart_tx
        MOVLW   'm'
        CALL    uart_tx
        MOVLW   ' '
        CALL    uart_tx
        MOVLW   'r'
        CALL    uart_tx
        MOVLW   'u'
        CALL    uart_tx
        MOVLW   'n'
        CALL    uart_tx
        MOVLW   't'
        CALL    uart_tx
        MOVLW   'i'
        CALL    uart_tx
        MOVLW   'm'
        CALL    uart_tx
        MOVLW   'e'
        CALL    uart_tx
        MOVLW   0x0D
        CALL    uart_tx
        MOVLW   0x0A
        CALL    uart_tx
        RETURN

; ------------------------------------------------------------
; send_status_line
;
; Prints:
;   tick=0xHHLL on=N btn=N press=0xHHLL mode=N pwm=N\r\n
;
; It snapshots TICK and PRESS with interrupts briefly disabled so the
; 16-bit values are internally consistent.
; ------------------------------------------------------------

send_status_line:
        ; Snapshot 16-bit values atomically.
        BCF     INTCON, GIE_BIT
        MOVF    TICK_LO, W
        MOVWF   SNAP_TICK_LO
        MOVF    TICK_HI, W
        MOVWF   SNAP_TICK_HI
        MOVF    PRESS_LO, W
        MOVWF   SNAP_PRESS_LO
        MOVF    PRESS_HI, W
        MOVWF   SNAP_PRESS_HI
        BSF     INTCON, GIE_BIT

        ; "tick=0x"
        MOVLW   't'
        CALL    uart_tx
        MOVLW   'i'
        CALL    uart_tx
        MOVLW   'c'
        CALL    uart_tx
        MOVLW   'k'
        CALL    uart_tx
        MOVLW   '='
        CALL    uart_tx
        MOVLW   '0'
        CALL    uart_tx
        MOVLW   'x'
        CALL    uart_tx

        MOVF    SNAP_TICK_HI, W
        CALL    print_hex8
        MOVF    SNAP_TICK_LO, W
        CALL    print_hex8

        ; " on="
        MOVLW   ' '
        CALL    uart_tx
        MOVLW   'o'
        CALL    uart_tx
        MOVLW   'n'
        CALL    uart_tx
        MOVLW   '='
        CALL    uart_tx

        ; Print logical heartbeat LED on state.
        ; RB1 is active-low, so PORTB.RB1 = 0 means on=1.
        BTFSC   PORTB, HEARTBEAT_LED
        GOTO    print_on_zero
        MOVLW   '1'
        GOTO    print_on_done
print_on_zero:
        MOVLW   '0'
print_on_done:
        CALL    uart_tx

        ; " btn="
        MOVLW   ' '
        CALL    uart_tx
        MOVLW   'b'
        CALL    uart_tx
        MOVLW   't'
        CALL    uart_tx
        MOVLW   'n'
        CALL    uart_tx
        MOVLW   '='
        CALL    uart_tx

        ; Print debounced button state.
        BTFSC   BUTTON_STATE, BTN_STABLE_BIT
        GOTO    print_btn_one
        MOVLW   '0'
        GOTO    print_btn_done
print_btn_one:
        MOVLW   '1'
print_btn_done:
        CALL    uart_tx

        ; " press=0x"
        MOVLW   ' '
        CALL    uart_tx
        MOVLW   'p'
        CALL    uart_tx
        MOVLW   'r'
        CALL    uart_tx
        MOVLW   'e'
        CALL    uart_tx
        MOVLW   's'
        CALL    uart_tx
        MOVLW   's'
        CALL    uart_tx
        MOVLW   '='
        CALL    uart_tx
        MOVLW   '0'
        CALL    uart_tx
        MOVLW   'x'
        CALL    uart_tx

        MOVF    SNAP_PRESS_HI, W
        CALL    print_hex8
        MOVF    SNAP_PRESS_LO, W
        CALL    print_hex8

        ; " mode="
        MOVLW   ' '
        CALL    uart_tx
        MOVLW   'm'
        CALL    uart_tx
        MOVLW   'o'
        CALL    uart_tx
        MOVLW   'd'
        CALL    uart_tx
        MOVLW   'e'
        CALL    uart_tx
        MOVLW   '='
        CALL    uart_tx
        MOVF    HEARTBEAT_MODE, W
        ADDLW   '0'
        CALL    uart_tx

        ; " pwm="
        MOVLW   ' '
        CALL    uart_tx
        MOVLW   'p'
        CALL    uart_tx
        MOVLW   'w'
        CALL    uart_tx
        MOVLW   'm'
        CALL    uart_tx
        MOVLW   '='
        CALL    uart_tx
        MOVF    PWM_MODE, W
        ADDLW   '0'
        CALL    uart_tx

        MOVLW   0x0D
        CALL    uart_tx
        MOVLW   0x0A
        CALL    uart_tx
        RETURN

; ------------------------------------------------------------
; print_hex8
;
; Input:
;   W = byte
;
; Output:
;   two uppercase ASCII hex characters over UART.
; ------------------------------------------------------------

print_hex8:
        MOVWF   HEX_TMP

        SWAPF   HEX_TMP, W
        ANDLW   0x0F
        CALL    hex_nibble_to_ascii
        CALL    uart_tx

        MOVF    HEX_TMP, W
        ANDLW   0x0F
        CALL    hex_nibble_to_ascii
        CALL    uart_tx

        RETURN

; ------------------------------------------------------------
; hex_nibble_to_ascii
;
; Input:
;   W = low nibble 0..15
; Output:
;   W = ASCII '0'..'9','A'..'F'
;
; ------------------------------------------------------------

hex_nibble_to_ascii:
        ANDLW   0x0F
        MOVWF   NIBBLE_TMP
        MOVLW   0x01
        MOVWF   PCLATH
        MOVF    NIBBLE_TMP, W
        ADDWF   PCL, F

        RETLW   '0'
        RETLW   '1'
        RETLW   '2'
        RETLW   '3'
        RETLW   '4'
        RETLW   '5'
        RETLW   '6'
        RETLW   '7'
        RETLW   '8'
        RETLW   '9'
        RETLW   'A'
        RETLW   'B'
        RETLW   'C'
        RETLW   'D'
        RETLW   'E'
        RETLW   'F'

; ------------------------------------------------------------
; uart_tx
;
; Input:
;   W = byte to transmit
;
; Output:
;   RB0 bit-banged UART TX, 4800 baud-ish, 8N1, idle high.
;
; Interrupts are disabled during each byte so the timing is not disturbed
; by Timer0 ISR jitter. This means Timer0 ticks may be skipped during long
; strings, which is acceptable for this debug/runtime milestone.
; ------------------------------------------------------------

uart_tx:
        MOVWF   TXBYTE

        ; Disable interrupts for clean UART timing.
        BCF     INTCON, GIE_BIT

        ; Start bit = 0.
        BCF     PORTB, TX_BIT
        CALL    bit_delay

        ; 8 data bits, LSB first.
        MOVLW   .8
        MOVWF   BITCNT

uart_bit_loop:
        BTFSC   TXBYTE, 0
        GOTO    uart_send_one

uart_send_zero:
        BCF     PORTB, TX_BIT
        GOTO    uart_bit_set

uart_send_one:
        BSF     PORTB, TX_BIT

uart_bit_set:
        CALL    bit_delay
        BCF     STATUS, C_BIT
        RRF     TXBYTE, F
        DECFSZ  BITCNT, F
        GOTO    uart_bit_loop

        ; Stop bit = 1.
        BSF     PORTB, TX_BIT
        CALL    bit_delay

        ; Re-enable interrupts.
        BSF     INTCON, GIE_BIT
        RETURN

; ------------------------------------------------------------
; bit_delay
;
; Delay loop used by UART TX
; At a 1 MHz external clock this produced the best practical ~4800 baud
; output for the AD3/logic-analyzer UART capture setup.
; ------------------------------------------------------------

bit_delay:
        MOVLW   .13
        MOVWF   BAUD_DLY

bit_delay_loop:
        DECFSZ  BAUD_DLY, F
        GOTO    bit_delay_loop
        RETURN

        END
