; *********************************************************** ;
;			Final Project	                      ;
;							      ;         			      
; *********************************************************** ;

    processor 16f1789
    #include "16bits.inc"
    #include "p16f1789.inc"
    ;#include "utility.inc"
  ; CONFIG1
; __config 0x2F82
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_ON & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_ON
; CONFIG2
; __config 0x1EFF
 __CONFIG _CONFIG2, _WRT_OFF & _VCAPEN_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF
 
    CLEAR_FLAG_TMR1 macro
	banksel PIR1
	bcf   PIR1, TMR1IF ; Clear interrupt flag
	endm
    CLEAR_CAPTURE_FLAG macro
	banksel PIR1
	bcf PIR1, CCP1IF
	endm
    CLEAR_FLAG_TMR0 macro
	bcf INTCON, TMR0IF
	endm
    CLEAR_FLAG_ADC macro
        movlb 0x00
	bcf   PIR1, 6
	endm
    ADC_ON macro
	movlb 0x01
	bsf   ADCON0, 0 ; Enables ADC
	endm
    ADC_OFF macro
	movlb 0x01
	bcf   ADCON0, 0 ; Diseable ADC
	endm
	
    RC3_ON macro
	banksel LATC
	bsf LATC, 3
	endm
	
    
    RC3_OFF macro
	banksel LATC
	bcf LATC, 3
	endm
	
   RC4_OFF macro
	banksel LATC
	bcf LATC, 4
	endm
   
    RC4_ON macro
	banksel LATC
	bsf LATC, 4
	endm
    
	
    RESET_TMR0_SAMPLING macro
 
	banksel TMR0
	clrf TMR0
	endm
	
    RESET_COUNTER macro
	movlw d'80' ; 10 us
	movwf counter
	endm
; That is where the MCU will start executing the program (0x00)
	org 	0x00
	nop
	goto START		    ; jump to the beginning of the code

; Interrupt routine	
    org 0x04
    ;goto CAPTURE_INTERRUPT
    movlb 0x00
    btfss INTCON, TMR0IF ; test if interrupt flag of timer 0 is set
    goto ADC_interrupt
    RESET_TMR0_SAMPLING
    CLEAR_FLAG_TMR0
    RESET_COUNTER ; 2 ins
    ADC_ON ; 2 ins
    call WaitAquisition ; 12 us
    movlb 0x01 ; 1 ins
    bsf   ADCON0, 1 ; Start an ADC conversion cycle ; 1 ins
Conversion:
    btfsc ADCON0, 1 ; is conversion done ?
    goto  Conversion
ADC_interrupt:
    movlb 0x00
    btfss PIR1, 6 ; 2 or 1 ins
    goto CAPTURE_INTERRUPT
    CLEAR_FLAG_ADC ; 2 ins
    ADC_OFF ; 2 ins
    movlb 0x01 ; 1 ins
    ;COMP16 ADRESL, ref ; 5 ins
    MOVLW   b'00000001'             ;  Compare the High Byte First
    SUBWF  ADRESH, W
    BTFSS  STATUS, Z              ;  If Result of High Byte Compare
    GOTO   Skip                   ;   is Equal to Zero, Then Check
    MOVLW   b'00010000'          ;   the Second
    SUBWF  ADRESL, W
    Skip:
    btfss STATUS, C ; if C = 1 ADRES>= ref ; 2 ins
    goto INCREASE_DUTY ; 2 ins
    movlb 0x05 ; 1 ins 
    decf CCPR2L, F ; 1 ins
     ; 2 ins
    retfie 
INCREASE_DUTY:
    movlb 0x05
    incf CCPR2L, F
    retfie ; with saturation
CAPTURE_INTERRUPT:
    banksel PIR1
    btfss PIR1, CCP1IF 
    retfie
    call INV_CCP1M_0
    CLEAR_CAPTURE_FLAG
    ; test if falling edge 0100 or rising edge 0101 : 
    ; if we are looking for falling edge, then retfie
    ; (the timer 1 will be incrementing)
    ; Otherwise, if we are looking for rising edge, then
    ; desactivate and reset TMR1 and retfie
    banksel CCP1CON
    btfss CCP1CON, 0
    goto START_TIMER1
    ; look which ultrasound sensor is working by testing bit0 
    ; of GENERAL variable
    btfss GENERAL, 0
    goto SENSOR0_IS_WORKING
SENSOR1_IS_WORKING: ; then make Distance1 = CCPR1H
    banksel CCPR1H  ; taking the low byte is useless (2cm of errors)
    ;banksel TMR1H
    movf CCPR1H, W
    movwf Distance1
    call RESET_DESACTIVATE_TMR1
    retfie
SENSOR0_IS_WORKING: ; then make Distance0 = CCPR1H
    banksel CCPR1H  ; taking the low byte is useless (2cm of errors)
    ;banksel TMR1H
    movf CCPR1H, W
    movwf Distance0
    call RESET_DESACTIVATE_TMR1
    retfie
START_TIMER1:
    banksel T1CON
    bsf T1CON, TMR1ON
    retfie
    


;BEGINNING OF THE PROGRAM
START:
    call	initialisation      ; initialisation routine configuring the MCU
    goto	main_loop           ; main loop

;INITIALISATION
initialisation:
    ; configure clock
    ;configuration of the GPIO
    
    ; Configure PORTB
    banksel TRISB
    movlw b'11111111'
    movwf TRISB
    banksel ANSELB
    clrf ANSELB

    ; Configure PORTD
    banksel TRISD
    movlw b'11111111'
    movwf TRISD
    banksel ANSELD
    movlw b'00000010'
    movwf ANSELD ; RD1 = Read V_buck
    banksel WPUD
    bcf WPUD, 1 ; disable weak pull-up on RD1

    ; Configure PORTC
    banksel TRISC
    movlw b'00000100' ; RC output expect RC2 = input ; capture mode :D
    movwf TRISC
    banksel ANSELC
    clrf ANSELC

    ; Configure PORTE
    banksel TRISE
    clrf TRISE
    banksel ANSELE
    clrf ANSELE

;configuration of clock - ? frequency - ? source
    movlb	0x01
    movlw	b'11111100'         ; 16 MHz
    movwf	OSCCON		    ; configure oscillator (cf datasheet SFR OSCCON)
    movlw	b'00000000'	    
    movwf	OSCTUNE		    ; configure oscillator (cf datasheet SFR OSCTUNE)

; Configure + initialize TMR1 for counting the longer of pulse .It cannot can exceed in practice 60.000 since
; in practice the max pulse is 30 ms = 60.000. Normally it won't reach 2^16=65.536 (overflow)
    banksel TMR1H
    clrf TMR1H
    clrf TMR1L
    banksel T1CON
    movlw b'00010001' ; Prescaler 1:2 + disable initially timer 1 + works at Fosc/4 = 16/4 = 4Mhz
    movwf T1CON	  ; Permet de prendre toute la pulse
    banksel T1CON
    bcf T1CON, TMR1ON ; le timer 1 commence à compter
; Setup TMR0
    banksel TMR0
    clrf TMR0
    bsf INTCON, TMR0IE
    bcf INTCON, TMR0IF
    bsf INTCON, IOCIF
    banksel OPTION_REG
    movlw b'10000011' ; prescalar 1:4 on TMR0 (overflow after 1066 ins)
    movwf OPTION_REG
    ; Interrupt declaration for TMR1

    banksel PIE1
    movlw	b'01000000' ; + Enables the ADC interrupt
    movwf	PIE1
    bsf PIE1, CCP1IE
    bsf	INTCON, GIE
    bsf	INTCON, PEIE


    ; Configure TMR2 for use with PWM module
    banksel PIR1
    bcf	PIR1,1 ; clear TMR2IF interrupt flag bit of the PIR register
    movlw	b'00000100'
    movwf	T2CON ; 1:1 postsclaller and 1 prescaler

    ; Setup PWM BUCK

    banksel PR2
    movlw	d'255'
    movwf	PR2
    banksel CCP2CON
    movlw	b'00001100' ; set PWM mode and the PWM duty cycle Least significant bits '00'
    movwf	CCP2CON
    movlw	b'01111111'
    movwf	CCPR2L      ; initialize the duty cycle to 0
    banksel APFCON1
    bcf APFCON1, CCP2SEL ; use RC1 instead of RB3

    ; Setup PWM Motor
    banksel APFCON2
    bcf APFCON2, CCP3SEL ; use RE0 instead of RB5
    banksel CCP3CON
    movlw b'00001100'
    movwf CCP3CON ; set PWM of motor to 0
    clrf CCPR3L

    ; CAPTURE ULSTRA-SOUND
    banksel APFCON1
    bsf APFCON1, CCP1SEL ; CCP1SEL = 1 : RB0 / 0 : RC2 (2 ultrasounds)
    banksel CCP1CON
    movlw b'00000101'
    movwf CCP1CON ; 0101 : capture every rising edge, 0100 : caoture every falling edge
    banksel PIR1
    bcf PIR1, CCP1IF
    banksel PIE1
    bsf PIE1, CCP1IE
    ; ADC parameters

    banksel ADCON0
    movlw b'11010100' ; use channel 21 + 10 bits conversion + 2's representation :D
    movwf ADCON0
    movlw b'11000000' ; Vref+ and Vref- to VDD, VSS, ADC conversion clock = Fosc/4, sign magnitude result
    movwf ADCON1
    movlw b'00001111'
    movwf ADCON2
; General Purpose ram Bank 0 for storing current PWM value of 2 bytes

    ; Variables declaration for PID computation : Common Ram location at 70h
    cblock 70h
    ref          : 2 ; 2 bytes is enough for ref : = 273 = 111h = b'0000000100010001'
    counter      : 1 ; 1 byte : initialize to 64 : use for acquisition time ADC
    GENERAL      : 1 ; general purpose variable
    counterms    : 1
    Distance0    : 1 ; 1 byte for computing distances (values of TMR1)
    Distance1    : 1 ; idem 
    Difference   : 1 ; Difference = Distance1-Distance0
    counti : 1
    endc

    ; Set the reference to 442h
    movlw b'00010000'  ; 100010000
    movwf ref ; Lower byte
    movlw b'00000001'
    movwf ref+1 ; Upper byte

    ; Initialize counter  
    movlw b'01100100' ; 64
    movwf counter
    clrf counterms
    ; Initialize all variables to 0
    clrf GENERAL
    clrf Distance1
    clrf Distance0
    clrf Difference
    movlw d'6'
    movwf counti
    return 

WaitAquisition: ; 12 us
    decfsz counter, 1
    ;btfss STATUS, 2 ; test Zero bit : if counter == 0 skip next instruction
    goto WaitAquisition 
    return
WAIT_7_6_MS:
    movlw b'11111111'
    movwf counterms
    iterate:
    call SEND_SIGNAL
    decfsz counterms, 1
    goto iterate
    return
WAIT_23_MS:
    call WAIT_7_6_MS
    call WAIT_7_6_MS
    call WAIT_7_6_MS
    call WAIT_7_6_MS
    call WAIT_7_6_MS
    call WAIT_7_6_MS
    return
MOTOR_DIR_0:
    banksel APFCON
    bcf APFCON, CCP1SEL
    banksel LATE
    bcf LATE, LATE1
    bsf LATE, LATE2
    return
MOTOR_DIR_1:
    banksel APFCON
    bsf APFCON, CCP1SEL
    banksel LATE
    bcf LATE, LATE2
    bsf LATE, LATE1
    return
INV_CCP1M_0:
    ; passe successivement de falling_edge, rising_edge
    banksel CCP1CON
    movlw b'11111110'
    xorwf CCP1CON,1
    comf  CCP1CON,1
    return

INV_GENERAL_1:
    movlw b'11111101'
    xorwf GENERAL,F
    comf  GENERAL,F
    return
    
RESET_DESACTIVATE_TMR1:
	
    banksel T1CON ;1 instr
    bcf   T1CON, TMR1ON ; Desactivate timer 1
    banksel TMR1L
    clrf TMR1L
    clrf TMR1H
    return
    
SEND_SIGNAL:
   
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    
    return
    
;MAIN LOOP
main_loop:
    ;;;;;;;;;;;;;;;;;; Manage the feedback of the projector ;;;;;;;;;;;;;;;;;;
    banksel PORTB
    btfss PORTB, 4
    goto DO_NOTHING
    banksel OSCCON
    movlw	b'11111000'         ; 16 MHz
    movwf	OSCCON		    ; configure oscillator (cf datasheet SFR OSCCON)
    banksel OSCTUNE
    movlw	b'00000000'	    
    movwf	OSCTUNE		    ; configure oscillator (cf datasheet SFR OSCTUNE)
    decfsz counti, F
    goto endi
    movlw d'255'
    movwf Distance0
    movwf Distance1
    movlw d'6'
    movwf counti
    endi:
    RC4_ON
    bcf GENERAL, 0 ; sensor 0 sending signal
    call SEND_SIGNAL ; 240 instructions for 15 us
    RC4_OFF
    call WAIT_23_MS
    RC3_ON
    bsf GENERAL, 0 ; sensor 1 sending signal
    call SEND_SIGNAL ; 240 instructions for 15 us
    RC3_OFF
    call WAIT_23_MS
    ; if |Difference| < 15 then do nothing else ...
    bcf STATUS, C
    ; If Distance1 < Distance0 => C flag = 0
    movf Distance0, W
    subwf Distance1, W
    movwf Difference
    btfsc STATUS, C
    goto superior
inferior:
    ; Difference is negative (2's complement) => take absolute value : complement + increment
    comf Difference, F
    incf Difference, F
    ;;;;;;;;;;;;;;;; test if difference < 15 ;;;;;;;;;;;;;;;;
    movlw d'15'
    subwf Difference, W ; W
    btfss STATUS, C ; if diff > 15 => C = 1
    goto DO_NOTHING
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    bcf STATUS, C
    ;rrf Difference, W ; decale 1 bit vers droite
    banksel CCPR3L
    movwf CCPR3L
    movlw d'170'
    movwf CCPR3L
    call MOTOR_DIR_1
    goto main_loop
superior:
    ;;;;;;;;;;;;;;;; test if difference < 15 ;;;;;;;;;;;;;;;;
    movlw d'15'
    subwf Difference, W ; W
    btfss STATUS, C ; if diff > 15 => C = 1
    goto DO_NOTHING
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ; Difference is positive
    bcf STATUS, C
    ;rrf Difference, W
    banksel CCPR3L
    movwf CCPR3L
    movlw d'170'
    movwf CCPR3L
    call MOTOR_DIR_0
    goto main_loop
DO_NOTHING:
    banksel CCPR3L
    clrf CCPR3L
    goto main_loop
    END