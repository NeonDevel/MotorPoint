;;======================================================================;;
;;			MotorPoint					;;
;;======================================================================;;
;;									
;; Program:         MotorPoint -- DCC motor controlled point		
;; Code:            Arkadiusz Hahn	
;; Platform:        Microchip PIC12F675, internal oscilator 4 Mhz				
;;
;; Date:            27.12.2015						
;; LastDate:        18.02.2016						
;;									
;;======================================================================;;
;
;
; Minimal external components, uses internal oscilator at 4 MHz, 
;
; This program is distributed as is but WITHOUT ANY WARRANTY
; I hope you enjoy!!
;
; Revisions:
; 12.12.2015	Start of writing code, based on ServoPoint by Paco Cañada   
; 19.12.2015  basic operation - decoding dcc accessory command.
; - SWITCH pressed >2.5s - enters PERIOD and SPEED programming mode
; - SWITCH pressed >5s -   enters ADDRESS programming mode   
; - during address programming last bit of second dcc command byte -
;     coil select is ignored.
; - added two buttons UP and DOWN connected to ACD converter on pin ANI3
; Voltage values on AIN3 - analog keyboard with UP/DOWN keys
; none: 0xff, UP: 0xAE=d174, Down: 0x7F = d127, DU&DOWN: 0x67=d103
; Keyboard voltage thresholds: released=190,UP=150,DOWN=115,UP&DOWN=90
; Working time PERIOD ranges: 0.5s,1s,1.5s,2s,2.5s,infinity. 
; SPEED controlled by PWM width. Range: 10%-90% with step 10%
; Pressing UP&DOWN keys in SPEED and PERIOD programming mode sets 
; default value of 50% and 0.5s respectively
; In address programming mode default value  is address of 0005 
;
; 2016.02.20 First release
 

; ----- Definitions
#define		__VERNUM	D'1'
#define		__VERDAY	 0x20
#define		__VERMONTH 0x02
#define		__VERYEAR	 0x16


  LIST	   p=12F675	; target processor

	errorlevel -305,-302

  #include p12F675.inc

	__CONFIG  _BODEN_ON & _CP_OFF & _WDT_ON & _MCLRE_OFF & _PWRTE_OFF & _INTRC_OSC_NOCLKOUT 

	; Make sure that internal osc. is calibrated
	; Value has to be read before reprogramming the device.

; --- Constant values

FXTAL   equ	D'4000000'		; internal oscilator

GP_TRIS     equ 0x1C			; GP2,GP3: inputs, GP4: analog input
ANSEL_INI   equ 0x28      ; GP4/AN3 pin - as analog input  ADCS2:0= 010 -> 32/Tosc = 8us conversion time
ADCON0_INI  equ 0x0C      ; AN3 connected to Sample&Hold , VDD - as voltage reference, result Left justified
OPTION_INI  equ 0x88			; Option register: no pull-up, falling GP2, no prescaler, wdt 1:1
WPU_INI     equ 0x23			; Weak pull-up enable on outputs GP0,GP1,GP5. default, no pull-ups

INTC_INI    equ	0xD0			; GIE, INTE enable, PEIE enable
;PIE1_INI    equ	0x01			; interrupt TMR1 
PIE1_INI   equ	0x41			; interrupt TMR1, ADIE 
POSITION_SAVE_DELAY equ 0x64   ; time delay x 20ms to  save current position in eeprom

; threshold voltage levels corresponding to the button held down
THR_K1  equ d'190'   
THR_K2  equ d'150'
THR_K12 equ d'115'
THR_KERR equ d'90'
DEBOUNCE_COUNT equ 4

#define		PWM     GPIO,0        ; PWM output 
#define		DIR     GPIO,1        ; DIR output
#define		DCCIN   GPIO,2      ; DCC input pin
#define		SWITCH	GPIO,3    ; Move/Programm switch (GPIO,3 only input)
#define		LED	    GPIO,5        ; Prog LED 
#define   KBD     GPIO,4        ; ANI3


; --- EEPROM Section

#define		EE_INI		0x00

EE_ADDR1H	 equ	EE_INI+0x00
EE_ADDR1L	 equ	EE_INI+0x01
EE_PERIOD1 equ	EE_INI+0x02
EE_SPEED1	equ	EE_INI+0x03

EE_OUT		equ	EE_INI+0x7F		; saved outputs


; ----- Variables

; --- Internal RAM Section

#define		RAMINI0		0x020		; 64 bytes of RAM

INT_W		equ	RAMINI0+0x00		; interrupt context registers
INT_STAT	equ	RAMINI0+0x01

SHIFT0		equ	RAMINI0+0x02
DATA1		equ	RAMINI0+0x03   ; DCC data buffer for incoming data
DATA2		equ	RAMINI0+0x04
DATA3		equ	RAMINI0+0x05
DATA4		equ	RAMINI0+0x06

PREAMBLE	equ	RAMINI0+0x08
DCCSTATE	equ	RAMINI0+0x09
DCCBYTE		equ	RAMINI0+0x0A

EEDATA0		equ	RAMINI0+0x0B		; EEPROM shadow variables
EEADR0		equ	RAMINI0+0x0C


SRVADRH1	equ	RAMINI0+0x10		; accessory decoder address
SRVADRL1	equ	RAMINI0+0x11		; 
PERIOD1		equ	RAMINI0+0x12		; motor active period 1-5s; n pulses * 20ms 
SPEED1		equ	RAMINI0+0x13		; PWM pulse width %   n*100*1us

PULSEH	equ	RAMINI0+0x14   ; calculated PWM pulse width = 100 * SPEED1
PULSEL	equ	RAMINI0+0x15

SRV1CNT		equ	RAMINI0+0x20 ; PWM pulses count to reach time PERIOD; 
PROGRAMMING_PHASE equ	RAMINI0+0x24	; current programming phase
POSITION	equ	RAMINI0+0x23		  ; current position flags  0=> straight, 1=> diverging
PWM_PHASE		  equ	RAMINI0+0x24	; current PWM phase 0-Active,1-pasive
POSITION_SAVE_CNT	equ	RAMINI0+0x25 ; downcouter - on zero save position in eeprom 

SPACE_H		equ	RAMINI0+0x26		; spacing duration - pasive PWM phase
SPACE_L		equ	RAMINI0+0x27

STATE equ RAMINI0+0x28      ; state of decoder (position,moving,programming)

DEBOUNCE1	equ	RAMINI0+0x2A		; key debouncing
DEBOUNCEP	equ	RAMINI0+0x2B		; PROG key debounce
TIMER		equ	RAMINI0+0x2C		; for prescaler prog key debouncing
FLSH_PRE	equ	RAMINI0+0x2D	; flash prescaler
FLSH1		equ	RAMINI0+0x2E		; flash sequence
FLSH2		equ	RAMINI0+0x2F		; 

FLAGS		equ	RAMINI0+0x30

COUNT		equ	RAMINI0+0x32
TEMPL		equ	RAMINI0+0x33
TEMPH		equ	RAMINI0+0x34

DATABF1		equ	RAMINI0+0x35   ; DCC data buffer for decoding
DATABF2		equ	RAMINI0+0x36
DATABF3		equ	RAMINI0+0x37
DATABF4		equ	RAMINI0+0x38
KEY_ADC		equ	RAMINI0+0x39

DEB_AKEY_NO	equ	RAMINI0+0x3A  ;analog key debouncing
DEB_AKEY_1	equ	RAMINI0+0x3B
DEB_AKEY_2	equ	RAMINI0+0x3C
DEB_AKEY_12	equ	RAMINI0+0x3D
AKEY_STATE equ	RAMINI0+0x3E



EEPTR		equ	RAMINI0+0x3F		; Page register


; --- Flags
;#define POSITION_STATE STATE,0
#define MOVING_STATE  STATE,1
#define PROGRAMMING_STATE  STATE,2
#define ADR_PROG_STATE STATE,3

;;  analog keyboard 
#define AKEY_UP AKEY_STATE,0
#define AKEY_DOWN AKEY_STATE,1
#define AKEY_UPDOWN AKEY_STATE,2


#define		NEW_PACKET	FLAGS,0		; New 3 byte packet received
#define		DCC4BYTE	FLAGS,3		; DCC command 4 bytes
#define		DO_PULSE	FLAGS,5		; do pulse, TMR1 end
#define   DO_ADC    FLAGS,6   ; ADC conversion end
#define		RESET_FLG	FLAGS,7		; reset packet



; --------------- Program Section --------------------------------------


		org	0x000

PowerUp:
		clrf	STATUS			; Bank 0 default
		clrf	INTCON			; Disable all interrupts
		clrf	PCLATH			; tables on page 0
		goto	INIT

; ----------------------------------------------------------------------

		org	0x004

Interrupt:
		movwf	INT_W			; save context registers		;1
		swapf	STATUS,w							;2
		movwf	INT_STAT							;3
		clrf	STATUS			; interrupt uses bank 0			;4

		btfss	PIR1,TMR1IF		; end of PWM pulse?			;+1
		goto	Int_DCC								;+2,3
   ; btfss DIR   ; set PWM = DIR  to deactivate output 	;+3 
   ; bcf	PWM			; yes, clear PWM pulse  +4
   ; btfsc DIR ;+5
   ; bsf	PWM   ;+6  clear PWM pulse in oposite direction
   ; btfss PROGRAMMING_STATE ;+7
   ; bcf LED   ; do not switch off in programming state +8
		bcf	PIR1,TMR1IF							;+9
		bsf	DO_PULSE							;+10

Int_DCC:
		btfss	INTCON,INTF		; External interrupt?			;+4,	+6
		goto	Int_ADC								;+5,6,	+7,8
		btfss	DCCIN								;5
		goto	Int_Low_Half							;6,7

Int_High_Half:
		movf	DCCSTATE,w							; 8
		addwf	PCL,f								; 9

		goto	Preamble							; 10,11
		goto	WaitLow
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadLastBit
		goto	EndByte1
		goto	EndByte2
		goto	EndByte3
		goto	EndByte4

Int_Low_Half:
		;movlw	d'256' - d'80'		; 77us: between 64us (one) and 90us (zero);8
    movlw	d'256' - d'77'		; 77us: between  64us (one) and 90us (zero);11
		movwf	TMR0								;9
		bcf	INTCON,T0IF		; clear overflow flag for counting	;10
		bcf	INTCON,INTF							;11
		bsf	STATUS,RP0							;12
		bsf	OPTION_REG,INTEDG	; next interrupt on rising edge GP2	;13
		goto	EndInt
    
EndHighHalf:
		bcf	INTCON,INTF							;21
		bsf	STATUS,RP0							;22
		bcf	OPTION_REG,INTEDG	; next interrupt on falling edge GP2	;23
    goto	EndInt
    
Int_ADC:
		btfss	PIR1,ADIF		; end of ACD conversion
		goto	EndInt					
    bcf	PIR1,ADIF   ; clear interrupt flag
    movf ADRESH,w
    movwf KEY_ADC
    ; end of acd conversion routine    
    bsf DO_ADC
EndInt:
		swapf	INT_STAT,w		; restore context registers		;24
		movwf	STATUS								;25
		swapf	INT_W,f								;26
		swapf	INT_W,w								;27
		retfie									;28,29


Preamble:
		btfss	NEW_PACKET		; wait until last decoded		;12
		incf	PREAMBLE,f		;					;13
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;14
		clrf	PREAMBLE		;					;15
		movlw	0xF6			; 10 preamble bits?			;16
		addwf	PREAMBLE,w		;					;17
		btfsc	STATUS,C		;					;18
		incf	DCCSTATE,f		; yes, next state			;19
		goto	EndHighHalf		;					;20,21
		

WaitLow:
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;12
		incf	DCCSTATE,f		; then state				;13
		clrf	DCCBYTE			;					;14
		clrf	PREAMBLE		;					;15
		clrf	DATA4			;					;16
		goto	EndHighHalf		;					;17,18


ReadBit:
		bsf	STATUS,C							;12
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;13
		bcf	STATUS,C							;14
		rlf	SHIFT0,f		; receiver shift register		;15
		incf	DCCSTATE,f		;					;16
		goto	EndHighHalf		;					;17,18
			
ReadLastBit:
		bsf	STATUS,C							;12
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;13
		bcf	STATUS,C							;14
		rlf	SHIFT0,f		; receiver shift register		;15
		incf	DCCBYTE,w							;16
		addwf	DCCSTATE,f							;17
		goto	EndHighHalf		;					;18,19

EndByte1:
		movlw	0x00			;					;12
		btfsc	INTCON,T0IF		; End bit=1, invalid packet		;13
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA1			;					;17
		incf	DCCBYTE,f		;					;18
		goto	EndHighHalf		;					;19,20

EndByte2:
		movlw	0x00			;					;12
		btfsc	INTCON,T0IF		; End bit=1, invalid packet		;13
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA2			;					;17
		incf	DCCBYTE,f		;					;18
		goto	EndHighHalf		;					;19,20


EndByte3:
		btfss	INTCON,T0IF		; End bit=1, end of packet		;12
		goto	EndByte3x		;					;13,14
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA3			;					;17
		incf	DCCBYTE,f		;					;18
		bsf	DCC4BYTE		;					;19
		goto	EndHighHalf		;					;20,21
EndByte3x:
		clrf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA3			;					;17
	  bsf	NEW_PACKET		;					;18
		bcf	DCC4BYTE		;					;19
		goto	EndHighHalf		;					;20,21

EndByte4:
		clrf	DCCSTATE		;					;12
		btfsc	INTCON,T0IF		; End bit=1, end of packet		;13
		goto	EndInt			; End bit=0, invalid packet		;14,15
		movf	SHIFT0,w		;					;15
		movwf	DATA4			;					;16
    bsf	NEW_PACKET		;					;17
		goto	EndHighHalf		;					;18,19


; ----------------------------------------------------------------------

; ----- Initialization

INIT:
		clrf	GPIO
		movlw	0x07
		movwf	CMCON			; set GP2:0 to digital I/O internal comparator Off
    movlw ADCON0_INI
    movwf ADCON0
		bsf	  STATUS,RP0		; bank 1
    movlw ANSEL_INI     
    movwf ANSEL     ; PIC12F675  Analog Select Register 
		movlw	GP_TRIS
		movwf	TRISIO
		call	0x3FF			; get OSCCAL value
   	movwf	OSCCAL
		movlw	WPU_INI			; pull-ups
		movwf	WPU
		clrf	IOC			; interrupt on change
		clrf	VRCON			; voltage reference off
		movlw	OPTION_INI		; Option register: no pull-up, falling GP2, no prescaler, wdt 1:1
		movwf	OPTION_REG
		movlw	PIE1_INI
		movwf	PIE1
		bcf	STATUS,RP0		; bank 0
		clrf	PIR1
		movlw	0x01			; Timer 1 on, 1:1  -  Fosc/4 = 1Mhz 
		movwf	T1CON


		movlw	0x20			; clear RAM
		movwf	FSR
ClearRAM:
		clrf	INDF
		incf	FSR,f
		movlw	0x60
		xorwf	FSR,w
		btfss	STATUS,Z
		goto	ClearRAM

		movlw	INTC_INI
		movwf	INTCON			; enable perypherial interupts - external interrupt GP2, ADC interupt
		call	LoadCV			; Load CV in SFR
		call	LoadOutputs	; set motor to last position
    

; ----------------------------------------------------------------------


MainLoop:
		btfsc	NEW_PACKET		; new packet?
		call	Decode			; yes, decode

		btfsc	DO_PULSE		; end of PWM pulse?
		call	DoMotor			; yes, next pulse
    clrwdt
		goto	MainLoop


DoMotor:
		bcf	DO_PULSE
		btfsc	INTCON,INTE		; disabled interrupts?
		goto	DoMotorJump
		clrf	DCCSTATE		; yes, clear for decoding
		bsf	INTCON,INTE		; re-enable interrupts
DoMotorJump:
		movf	PWM_PHASE,w
		andlw	0x03
		addwf	PCL,f
    goto PwmPhaseActive
    goto PwmPhasePasive
		clrf	PWM_PHASE			; prevents erroneus contents


	if ($ > d'255') 
		ERROR "  Tables exceded page 0.   If it works, why do you change it?   "
	endif

; ----------------------------------------------------------------------

PwmPhaseActive: 
    clrf	SPACE_H			; init spacing value (20ms)
		clrf	SPACE_L
		call	PulseMotor  ; 
   ; bcf	INTCON,INTE		; disable DCC interrupts for time accuracy
    btfss	MOVING_STATE
    goto PwmPhaseActive1
    ; DIR depends on POSITION, and PWM is counter
    btfsc	POSITION,0
    bsf DIR
    btfsc	POSITION,0
    bcf PWM
    btfss	POSITION,0
    bcf DIR
    btfss	POSITION,0
    bsf PWM
    btfsc SWITCH  ; do not set LED when switch is on
    bsf LED
PwmPhaseActive1:
    bsf	T1CON,TMR1ON		; run timer 1
		incf	PWM_PHASE,f   ; next phase on timer overflow
    return
    
PwmPhasePasive:  
    bcf	T1CON,TMR1ON
		movf	SPACE_H,w		; pulses length
		movwf	TMR1H
		movf	SPACE_L,w
		movwf	TMR1L
		movlw	0xE0			  ; 20ms time ( 0x10000 - 0x4E20 = 0xB1E0 )
		addwf	TMR1L,f
		btfsc	STATUS,C
		incf	TMR1H,f
		movlw	0xB1
		addwf	TMR1H,f	
    clrf	PWM_PHASE
		bsf	T1CON,TMR1ON		; run timer 1
    
    btfss DIR   ; set PWM = DIR  to deactivate output 	;+3 
    bcf	PWM			; yes, clear PWM pulse  +4
    btfsc DIR ;+5
    bsf	PWM   ;+6  clear PWM pulse in oposite direction
    btfss PROGRAMMING_STATE ;+7
    bcf LED   ; do not switch off in programming state +8
    
    btfss	MOVING_STATE
    goto	EndPWM
    ; count down  moving period
    movlw	0xff			; SRV1CNT is 0xFF do not count time
		xorwf	SRV1CNT,w
		btfsc	STATUS,Z
    goto	EndPWM
    movf	SRV1CNT,w  
		btfss	STATUS,Z   ; if SRV1CNT is 0x00 do not count time
    decfsz SRV1CNT,f
    goto	EndPWM
    bcf MOVING_STATE      ; PWM time period reached
		goto	EndPWM  

; ----------------------------------------------------------------------   
PulseMotor:
    bcf	T1CON,TMR1ON		; stop timer 1
    ;movlw 0x46    ;95% when reached
    movf PULSEH,w
    movwf TMR1H
    movf PULSEL,w
    movwf TMR1L

    movf	TMR1L,w			; correct spacing
		addwf	SPACE_L,f
		btfsc	STATUS,C
		incf	SPACE_H,f
		movf	TMR1H,w
		addwf	SPACE_H,f

		comf	TMR1L,f			; negative for incrementing
		comf	TMR1H,f
		movlw	0x01
		addwf	TMR1L,f
		btfsc	STATUS,C
		incf	TMR1H,f
    return
  

 ; ---------------------------------------------------------------------- 

EndPWM:		; every 20ms
    movf	POSITION_SAVE_CNT,w  
		btfss	STATUS,Z   ; if SRV1CNT is 0x00 do not count time
    decfsz POSITION_SAVE_CNT,f
    goto CheckKey
    ; save new position when couter reached zero
    movf	POSITION,w		; save new output state	
		movwf	EEDATA0
		movlw	EE_OUT
		call	SetParm
    return

; ----------------------------------------------------------------------
ChangePosition:
    movlw 0x01
    xorwf POSITION,f 
MoveToPosition:
    movf	PERIOD1,w		; set period
		movwf	SRV1CNT
    bsf MOVING_STATE 
    movlw POSITION_SAVE_DELAY  ; request to Save position
    movwf POSITION_SAVE_CNT  
    return
; ----------------------------------------------------------------------
    

CheckKey:    
		btfsc	SWITCH			; check program switch 
		goto	ReadInputSwitch
		decf	DEBOUNCE1,f		; *** for 2,5s   ; decrement twice: 20ms * 128 = 2,56s
		decfsz	DEBOUNCE1,f		; pressed, wait debounce time (2,5s)
		return
    goto	MainProg		; enter programming mode
ReadInputSwitch:
		movf	DEBOUNCE1,w		; short pressed?
		btfsc	STATUS,Z
  	return		; no		
Key_Change:
		clrf	DEBOUNCE1
    ; motor move
    call ChangePosition
		return

; ----------------------------------------------------------------------


MainProg:
    bsf PROGRAMMING_STATE
    bsf ADCON0,ADON     ; enable ADC for analog keyboard 
		bsf	LED			; LED on
    ;clrf PROGRAMMING_PHASE
    movlw 0x26     ;; 38*65ms = 2.5s  to enter address programming mode
    movwf DEBOUNCE1 ; in next 2.5s enter address programming mode.
    bsf ADCON0,GO_NOT_DONE  ; start conversion
MainProg1:  
    clrwdt  
    btfsc SWITCH			; wait to release 
    goto MainProg3
    btfss	DO_PULSE		; every 20ms
    goto MainProg1 
    bcf DO_PULSE 
    decfsz	DEBOUNCE1,f  
    goto MainProg1 
    ; address programming mode
MainProg2:
    clrwdt
    bcf LED   ; again LED is off  
    btfss	SWITCH			; wait to release
		goto	MainProg2
    bsf ADR_PROG_STATE    
    clrf EEPTR
    bsf	FLSH_PRE,1
		call	SetFlash
    goto ProgLoop
    
MainProg3:    
		clrf	EEPTR
    bsf EEPTR,1  ; SPEED programming
		bsf	FLSH_PRE,1
		call	SetFlash
ProgLoop:
    clrwdt
		btfss	DO_PULSE ; every 20ms		
		goto	ProgKey
    ; set timer period  - 20ms  ( 0x10000 - 0x4E20 = 0xB1E0 )
    movlw 0xE0
    movwf TMR1L
    movlw 0xB1
    movwf TMR1H
		bcf	DO_PULSE
		decfsz	FLSH_PRE,f		; prescaler flash
		goto	ProgADC
    movlw 0x08
		movwf	FLSH_PRE
		bcf	STATUS,C
		btfsc	FLSH1,7
		bsf	STATUS,C
		rlf	FLSH2,f
		rlf	FLSH1,f
		btfss	STATUS,C
		bcf	LED
		btfsc	STATUS,C
		bsf	LED
ProgADC:    
    ; Adc keyboard  - every 20ms
    btfss DO_ADC
    goto ProgKey
    bcf DO_ADC
    ;; compare adc value with thresholds
    movlw THR_K1
    subwf KEY_ADC,w  
    btfsc STATUS,C  
    goto KeyReleased   ;KEY_ACC>THR_K1
    clrf DEB_AKEY_NO
    movlw THR_K2
    subwf KEY_ADC,w  
    btfsc STATUS,C  
    goto KeyUp   ;KEY_ACC>THR_K2
    movlw THR_K12
    subwf KEY_ADC,w  
    btfsc STATUS,C  
    goto KeyDown   ;KEY_ACC>THR_K12
    movlw THR_KERR
    subwf KEY_ADC,w  
    btfsc STATUS,C  
    goto KeyUpDown   ;KEY_ACC>THR_KERR
KeyERR:    
    clrf DEB_AKEY_NO
    clrf DEB_AKEY_1
    clrf DEB_AKEY_2
    clrf DEB_AKEY_12
    bcf AKEY_UP
    bcf AKEY_DOWN
    bcf AKEY_UPDOWN
    goto KeyEnd
KeyReleased:
    bsf STATUS,C
    rlf DEB_AKEY_NO
    btfss DEB_AKEY_NO,DEBOUNCE_COUNT
    goto KeyEnd
    btfsc DEB_AKEY_1,DEBOUNCE_COUNT
    bsf AKEY_UP
    btfsc DEB_AKEY_2,DEBOUNCE_COUNT
    bsf AKEY_DOWN
    btfsc DEB_AKEY_12,DEBOUNCE_COUNT
    bsf AKEY_UPDOWN
    clrf DEB_AKEY_NO
    clrf DEB_AKEY_1
    clrf DEB_AKEY_2
    clrf DEB_AKEY_12
    goto KeyEnd
KeyUp:  
    btfsc DEB_AKEY_2,DEBOUNCE_COUNT   ;manage releasing keys
    goto KeyEnd
    btfsc DEB_AKEY_12,DEBOUNCE_COUNT
    goto KeyEnd
    bsf STATUS,C      ; key still pressed
    rlf DEB_AKEY_1
    goto KeyEnd
KeyDown:
    btfsc DEB_AKEY_12,DEBOUNCE_COUNT  ;manage releasing keys 
    goto KeyEnd
    clrf DEB_AKEY_1
    bsf STATUS,C        ; key still pressed
    rlf DEB_AKEY_2      
    goto KeyEnd
KeyUpDown:
    clrf DEB_AKEY_1
    clrf DEB_AKEY_2
    bsf STATUS,C
    rlf DEB_AKEY_12
    goto KeyEnd
KeyEnd:    
    bsf ADCON0,GO_NOT_DONE  ; start conversion
ProgKey:
		decfsz	TIMER,f			; debounce time
		goto	ProgNoKey
		bsf	TIMER,2
		bcf	STATUS,C		; check key
		btfsc	SWITCH
		bsf	STATUS,C
		rlf	DEBOUNCEP,w
		movwf	DEBOUNCEP
		xorlw	0xC0
		btfss	STATUS,Z
		goto	ProgNoKey
		movf EEPTR,w
    btfsc STATUS,Z
    goto EndProg   ; exit from address programing
    btfss	EEPTR,1
		incf	EEPTR,f
		incf	EEPTR,f
		call	SetFlash
		btfss	EEPTR,2
		goto	ProgLoop
EndProg:
		bcf ADCON0,GO_NOT_DONE  ; stop pending ADC conversion
    bcf LED
    bcf ADR_PROG_STATE
    bcf PROGRAMMING_STATE
    bcf ADCON0,ADON     ; dissable ADC converter 
		bsf	INTCON,INTE		; enable interrupts
		return

ProgNoKey:
ProgAKey:  
    btfsc	EEPTR,1
    goto ProgAKeyPeriodSpeed
    ;analog keyboard during address programming
    bcf AKEY_DOWN  ; action not defined 
    bcf AKEY_UP    ; action not defined
    btfss AKEY_UPDOWN
    goto ProgAkeyAddressEnd
    ; up and down pressed during address programming
    ; reset addres
    bcf AKEY_UPDOWN
    movlw 0x81
    movwf DATABF1
    movlw 0xF8
    movwf DATABF2
    goto	ProgSetAddr		; address
ProgAkeyAddressEnd:
		btfss	NEW_PACKET		; new packet?
		goto	ProgLoop
DecodeProg:
    ; copy DATA to decoder buffer
    movf DATA1,w
    movwf DATABF1
    movf DATA2,w
    movwf DATABF2
    movf DATA3,w
    movwf DATABF3
    movf DATA4,w
    movwf DATABF4
    
		bcf	NEW_PACKET		; prepare for next packet
		bcf	INTCON,INTE		; disable interrupts for more speed

		movf	DATABF1,w			; exclusive or check
		xorwf	DATABF2,w
		xorwf	DATABF3,w
		xorwf	DATABF4,w

		btfss	STATUS,Z		; valid packet?
		goto	ExitDecodeProg		; no, return

		movf	DATABF1,w			;'10AAAAAA'  '1AAADxxx' AAA:111 D:1 activate
		andlw	0xC0
		xorlw	0x80			;'10xxxxxx'? accessory operation
		btfsc	DATABF2,7
		btfss	STATUS,Z
		goto	ExitDecodeProg
		btfss	DATABF2,3			; activate?
		goto	ExitDecodeProg
		btfss	EEPTR,1
		goto	ProgSetAddr		; address
ProgPeriodSpeed:   ; not used
		decf	DATABF1,f			; '10AAAAAA'
		rlf	DATABF1,w
		movwf	TEMPL
		rlf	TEMPL,w			; 'AAAAAAxx'
		andlw	0xFC
		movwf	TEMPL	

		rrf	DATABF2,w			; '1AAA1CCD'
		andlw	0x03
		iorwf	TEMPL,f
		incf	TEMPL,w
		movwf	EEDATA0

		movf	EEPTR,w
		call	SetParm
		call	LoadCV
		bsf MOVING_STATE		; move to test
		movf	PERIOD1,w
		movwf	SRV1CNT
		goto	EndProg

ProgAKeyPeriodSpeed:
    btfss	EEPTR,0
    goto ProgAKeyPeriod
    ; Prog Speed
    btfss AKEY_UP
    goto  ProgAKeySpeedDown
    ; keyUp
    movlw d'180'    ;max 90%
    subwf SPEED1,w  
    btfsc STATUS,Z 
    goto ProgAKeySpeedEnd  
    movlw d'20'     ;+10%
    addwf SPEED1,w
    goto ProgAKeySetSpeedValue    
ProgAKeySpeedDown:   
    btfss AKEY_DOWN
    goto  ProgAKeySpeedUpDown
    movlw d'20'   ; min 10%
    subwf SPEED1,w  
    btfsc STATUS,Z 
    goto ProgAKeySpeedEnd  
    movlw d'20'   ; -10%
    subwf SPEED1,w
    goto ProgAKeySetSpeedValue      
ProgAKeySpeedUpDown:   
    btfss AKEY_UPDOWN
    goto  ProgAKeySpeedEnd
    movlw d'100'  ; 50%
ProgAKeySetSpeedValue:    
    movwf SPEED1
    movwf	EEDATA0
    movf	EEPTR,w
		call	SetParm
    call SetPulseWidth
ProgAKeySpeedEnd:    
    bcf AKEY_UP
    bcf AKEY_DOWN
    bcf AKEY_UPDOWN
    goto  ProgLoop
ProgAKeyPeriod:
    btfss AKEY_UP
    goto  ProgAKeyPeriodDown
    ; keyUp
    movlw d'255'    ;0xff = infinity
    subwf PERIOD1,w
    btfsc STATUS,Z 
    goto ProgAKeyPeriodEnd  
    movlw d'125'    ;max 2.5s
    subwf PERIOD1,w  
    btfsc STATUS,Z 
    goto ProgAKeyPeriodInf  ; set 0xFF 
    movlw d'25'     ;+0.5s
    addwf PERIOD1,w
    goto ProgAKeySetPeriodValue
ProgAKeyPeriodInf:
    movlw d'255'
    goto ProgAKeySetPeriodValue
ProgAKeyPeriodDown:   
    btfss AKEY_DOWN
    goto  ProgAKeyPeriodUpDown
    movlw d'255'    ;if (infinity)  set 2.5s
    subwf PERIOD1,w
    btfsc STATUS,Z 
    goto ProgAKeyPeriodMax 
    movlw d'25'   ;  min 0.5s
    subwf PERIOD1,w  
    btfsc STATUS,Z 
    goto ProgAKeyPeriodEnd  
    movlw d'25'   ; -0.5s
    subwf PERIOD1,w
    goto ProgAKeySetPeriodValue
ProgAKeyPeriodMax:
    movlw d'125'
    goto ProgAKeySetPeriodValue    
ProgAKeyPeriodUpDown:   
    btfss AKEY_UPDOWN
    goto  ProgAKeyPeriodEnd
    movlw d'25'  ; 0.5s 
ProgAKeySetPeriodValue:    
    movwf PERIOD1
    movwf	EEDATA0
    movf	EEPTR,w
		call	SetParm
    ;call SetPulseWidth
ProgAKeyPeriodEnd:    
    bcf AKEY_UP
    bcf AKEY_DOWN
    bcf AKEY_UPDOWN
    goto  ProgLoop
    
ProgSetAddr:
		movf	DATABF1,w
		movwf	EEDATA0
		movlw	EE_ADDR1H
		call	SetParm			; save address
		movf	DATABF2,w
    andlw 0xFE    ; make sure, address not depending on coil select bit D in '1AAA1CCD' 
		movwf	EEDATA0
		movlw	EE_ADDR1L
		call	SetParm			; save address
		call	LoadCV
		bsf	MOVING_STATE		; move to test
		movf	PERIOD1,w
		movwf	SRV1CNT
		goto	EndProg

ExitDecodeProg:
		bsf	INTCON,INTE		; enable interrupts
		goto	ProgLoop



SetFlash:
		btfss	EEPTR,1
		goto	FlashLED
		movlw	b'10100000'		 ; PERIOD  
		btfsc	EEPTR,0
		movlw	b'10101000'    ; SPEED
		movwf	FLSH1
		movwf	FLSH2
		return
FlashLED:
		movlw	b'11110000'   ; addres programing
		movwf	FLSH1
		movwf	FLSH2
		return


; ----------------------------------------------------------------------

Decode:
		bcf	NEW_PACKET		; prepare for next packet
		bcf	INTCON,INTE		; disable interrupts for more speed

		movf	DATA1,w			; exclusive or check
		xorwf	DATA2,w
		xorwf	DATA3,w
		xorwf	DATA4,w

		btfss	STATUS,Z		; valid packet?
		goto	ExitDecode		; no, return

; 'AAAAAAAA''DDDDDDDD''EEEEEEEE'		; 3 byte packet
;   DATA1     DATA2     DATA3

		movf	DATA1,w			; address = '00000000' ?
		btfsc	STATUS,Z
		goto	Broadcast
		andlw	0xC0
		xorlw	0x80			;'10xxxxxx'? accessory operation
		btfss	STATUS,Z
		goto	ExitDecode
		goto	Accessory

Accessory:
		bcf	RESET_FLG

AccSrv1:
		movf	SRVADRH1,w		; '10AAAAAA' of decoder 1
		xorwf	DATA1,w
		btfss	STATUS,Z
		goto	AccSrvEnd
		movf	SRVADRL1,w		; '1AAACDDD'
		xorwf	DATA2,w
		btfss	STATUS,Z
		goto	AccSrv1B
		btfsc	POSITION,0		; move motor
		goto  AccSrvEnd
		goto	AccSrv1Change
AccSrv1B:
		xorlw	0x01			; other position?
		btfss	STATUS,Z
		goto	AccSrvEnd ; not my address
		btfss	POSITION,0		; move motor
    goto  AccSrvEnd
AccSrv1Change:
    movlw 0x01
    xorwf POSITION,f 
    movf	PERIOD1,w		; set period
		movwf	SRV1CNT
    bsf	MOVING_STATE
    movlw POSITION_SAVE_DELAY  ; request to Save position
    movwf POSITION_SAVE_CNT  
   
AccSrvEnd:
		goto	ExitFunction


ExitFunction:
		bcf	RESET_FLG
		bsf	INTCON,INTE		; enable DCC interrupts
		return


; -----------------------------------------------------------------------

Broadcast:
		movf	DATA2,w			; reset packet?
		btfss	STATUS,Z
		goto	ExitDecode
;;		bcf	PROG_2X
		bsf	RESET_FLG

Clear:						; reset decoder

;;		bcf	SERVO			; servo disable

		bsf	INTCON,INTE		; enable interrupts
		return


ExitDecode:
		bcf	RESET_FLG
		bsf	INTCON,INTE		; enable interrupts
		return


;---------------------------------------------------------------------------

; Copy Address, Speed , and Period from eeprom to RAM
LoadCV:
		movlw	SRVADRH1		; first CV to read
		movwf	FSR
		movlw	0x00
		movwf	EEADR0
LoadCVNxt:
		movf	EEADR0,w
		call	EE_Read
		movwf	INDF
		movlw	SPEED1			; last CV to read
		xorwf	FSR,w
		btfsc	STATUS,Z
		goto	LoadCVEnd
		incf	FSR,f
		incf	EEADR0,f
		goto	LoadCVNxt
LoadCVEnd:

		return


;---------------------------------------------------------------------------

LoadOutputs:
		movlw	EE_OUT			; read saved outputs
		call	EE_Read
		movwf	POSITION			
		
    call SetPulseWidth   ; 100*Speed
    
		movf PERIOD1,w    ; pulses count for setting position 
		movwf	SRV1CNT
		bsf	MOVING_STATE
		return
; ----------------------------------------------------------------------
SetPulseWidth:    
    ;100 * SPEED1
    movf SPEED1,w
    movwf	TEMPL
    clrf	TEMPH			; do SPEED x 100 =  SPEED*(4+32+64)
    bcf	STATUS,C	
    rlf	TEMPL,f     ; x2
		rlf	TEMPH,f
    bcf	STATUS,C	
		rlf	TEMPL,f			; x4
    rlf	TEMPH,f
		movf TEMPL,w
		movwf PULSEL
    movf TEMPH,w
		movwf PULSEH
    
    bcf	STATUS,C	
    rlf	TEMPL,f     ; x8
		rlf	TEMPH,f
    bcf	STATUS,C	
		rlf	TEMPL,f			; x16
    rlf	TEMPH,f
    bcf	STATUS,C	
		rlf	TEMPL,f			; x32
    rlf	TEMPH,f
    movf TEMPL,w
		addwf PULSEL,f
    btfsc	STATUS,C
		incf	PULSEH,f
    movf TEMPH,w
		addwf PULSEH,f
    bcf	STATUS,C	
		rlf	TEMPL,f			; x64
    rlf	TEMPH,f
    movf TEMPL,w
		addwf PULSEL,f
    btfsc	STATUS,C
		incf	PULSEH,f
    movf TEMPH,w
		addwf PULSEH,f
    return
    
;----- Internal EEPROM routines ------------------------------------------------


EE_Read:
		bsf	STATUS,RP0		; w=ADR
		movwf	EEADR
		bsf	EECON1,RD
		movf	EEDATA,w
		bcf	STATUS,RP0
		return

SetParm:
		call	EE_Read			; w=ADR, EEDATA0=data. Write only changes
		xorwf	EEDATA0,w
		btfsc	STATUS,Z
		return
EE_Write:		
		movf	EEDATA0,w
		bsf	STATUS,RP0
		movwf	EEDATA
		bsf	EECON1,WREN
		bcf	INTCON,GIE
		movlw	0x55
		movwf	EECON2
		movlw	0xAA
		movwf	EECON2
		bsf	EECON1,WR
		bsf	INTCON,GIE
		bcf	EECON1,WREN
EEWrite0:
		btfsc	EECON1,WR
		goto	EEWrite0
		bcf	STATUS,RP0
		return

;--------------------------------------------------------------------


; ----- EEPROM default values


		org	0x2100


						;'10AAAAAA'1AAACDDD'
		dw	0x81			; address  (roco 0005)  
		dw	0xF8
		dw	d'25'			; period  50*20ms = 1s 
		dw	d'100'		; default speed  100us*100  = 10ms = PWM 50%

; addresing  NMRA 9.2.1 paragraph 410
; '10AAAAAA'1AAACDDD'   details   1 0 A5 A4 A3 A2 A1 A0   1 !A8 !A7 !A6 C D1 D0 S
; where C=1  (active) ;  D0 D1  select one of four outputs of the state S
;examples:
; adr  0x80F8  (roco 0001)   1000 0000 1111 1000  => A=000 00 0000,D=00,S=0  straight
; adr  0x81F8  (roco 0005) = 1000 0001 1111 1000  => A=000 00 0001,D=00,S=0  straight  
; adr  0x81FA  (roco 0006) = 1000 0001 1111 1010  => A=000 00 0001,D=01,S=0  straight
; adr  0x81FB  (roco 0006!)= 1000 0001 1111 1011  => A=000 00 0001,D=01,S=1  diverging


		org	0x2120

		dt	" Motor  "
		dt	" Point  "
		dt	"Arkadiusz Hahn "
		dt	(__VERDAY   >> 4)  +0x30
		dt	(__VERDAY   & 0x0F)+0x30,"/"
		dt	(__VERMONTH >> 4)  +0x30
		dt	(__VERMONTH & 0x0F)+0x30,"/"
		dt	(__VERYEAR  >> 4)  +0x30
		dt	(__VERYEAR  & 0x0F)+0x30

		org	0x217F

		dw	0x00			; last state

	end
