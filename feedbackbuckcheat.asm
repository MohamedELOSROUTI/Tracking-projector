; *********************************************************** ;
;        Feedback Control of the buck converter	              ;
;							      ;
;	      Kp = 0.5, Ki = 0.0625, Kd = 2                   ;
;                                			      ;
; *********************************************************** ;

    processor 16f1789
    #include 	"config3.inc"
    #include    "16bits.inc"
    
    
    CLEAR_FLAG_TMR1 macro
	movlb 0x00
	bcf   PIR1, 0 ; Clear interrupt flag
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
	
    RESET_TMR1_SAMPLING macro
	
    	movlb 0x00 ;1 instr
	bcf   T1CON, 0 ; Desactivate timer 1
	movlw b'11010100' ; 1 inst
	movwf TMR1L ; 1 instr
	movlw b'11111011' ; 1 instr
	movwf TMR1H ; 1 instr
	bsf   T1CON, 0 ; activate timer 1
	endm
	
    RESET_COUNTER macro
	movlw b'01000000'
	movwf counter
	endm
; That is where the MCU will start executing the program (0x00)
	org 	0x00
	nop
	goto    start		    ; jump to the beginning of the code

; Interrupt routine	
    org 0x04
    movlb 0x00
    btfss PIR1, 0 ; test if interrupt flag of timer 1 is set
    goto ADC_interrupt
    CLEAR_FLAG_TMR1
    ;RESET_TMR1_SAMPLING
    RESET_COUNTER
    ADC_ON
WaitAquisition:
    decf counter, 1
    btfss STATUS, 2 ; test Zero bit : if counter == 0 skip next instruction
    goto WaitAquisition 
    movlb 0x01
    bsf   ADCON0, 1 ; Start an ADC conversion cycle
Conversion:
    btfsc ADCON0, 1 ; is conversion done ?
    goto  Conversion
ADC_interrupt:
    movlb 0x00
    btfss PIR1, 6
    retfie
    CLEAR_FLAG_ADC
    ADC_OFF
    movlb 0x01
    ;COMP16 ADRESL, ref
    MOVLW   b'00000001'             ;  Compare the High Byte First
    SUBWF  ADRESH, W
    BTFSS  STATUS, Z              ;  If Result of High Byte Compare
    GOTO   Skip                   ;   is Equal to Zero, Then Check
    MOVLW   b'00010000'          ;   the Second
    SUBWF  ADRESL, W
    Skip: 
    btfss STATUS, C ; if C = 1 ADRES>= ref
    goto INCREASE_DUTY
    movlb 0x05
    DEC16 CCPR2L
    retfie
   
INCREASE_DUTY:
    movlb 0x05
    INC16 CCPR2L
    retfie ; with saturation
	
;BEGINNING OF THE PROGRAM
start:
	call	initialisation      ; initialisation routine configuring the MCU
	goto	main_loop           ; main loop

;INITIALISATION
initialisation:
    ; configure clock
    ;configuration of the GPIO
    
	; Configure PORTD
	movlb 0x01
	bsf TRISD, 1 ; make port RD1 as input 
	movlb 0x03
	bsf ANSELD, 1 ; make port RD1 as analog input
	movlb 0x04
	bcf WPUD, 1 ; disable weak pull-up on RD1
	
	; Configure PORTC
	movlb	0x01
	clrf	TRISC ; All pins of PORTC are output (including RC2)
	movlb	0x02
	movlw	b'00000100' ;RC2 =0 while RC0..RC7=0
	movwf	LATC

    ;configuration of clock - ? frequency - ? source
	movlb	0x01
	movlw	b'01111110'
	movwf	OSCCON		    ; configure oscillator (cf datasheet SFR OSCCON)
	movlw	b'00000000'	    
	movwf	OSCTUNE		    ; configure oscillator (cf datasheet SFR OSCTUNE)
    
    ; Configure + initialize TMR1
	movlb 0x00
	movlw b'11111011' ; TMR1 = 65268 in order to have overflow after 267 ins
	movwf TMR1H       ; The overflow frequency corresponds to 15 Khz for the sampling frequency : T = 2*Ts
	movlw b'11010100'
	movwf TMR1L      
	movlw b'00000001' ; Prescaler 1:1 + enable timer 1
	movwf T1CON
	
	; Interrupt declaration for TMR1
    
	movlb	0x01
	movlw	b'01000001' ; + Enables the ADC interrupt
	movwf	PIE1
	bsf	INTCON, 7
	bsf	INTCON, 6
	
	
	; Configure TMR2 for use with PWM module
	movlb	0x00
	bcf	PIR1,1 ; clear TMR2IF interrupt flag bit of the PIR register
	movlw	b'00000100'
	movwf	T2CON ; 1:1 postsclaller and 1 prescaler
	
	; Setup PWM
	
        movlb	0x00
        movlw	b'10000101' ;66
        movwf	PR2
        movlb	0x05
        movlw	b'00001100' ; set PWM mode and the PWM duty cycle Least significant bits '00'
        movwf	CCP2CON
	movlw	b'01000011'
	movwf	CCPR2L      ; initialize the duty cycle to 0
	; ADC parameters
	
	movlb 0x01
	movlw b'11000000' ; use channel 21 + 10 bits conversion + 2's representation :D
	movwf ADCON0
	movlw b'11000000' ; Vref+ and Vref- to VDD, VSS, ADC conversion clock = Fosc/4, sign magnitude result
	movwf ADCON1
	;movlw b'00001110'
	;movwf ADCON2
; General Purpose ram Bank 0 for storing current PWM value of 2 bytes
	
	; Variables declaration for PID computation : Common Ram location at 70h
	cblock 70h
	ref          : 2 ; 2 bytes is enough for ref : = 273 = 111h = b'0000000100010001'
	counter      : 1 ; 1 byte : initialize to 64 : use for acquisition time ADC
	endc
	
	; Set the reference to 442h
	movlw b'00010000'  ; 100010000
	movwf ref ; Lower byte
	movlw b'00000001'
	movwf ref+1 ; Upper byte
	
	; Initialize counter  
	movlw b'01100100' ; 64
	movwf counter
	; Initialize all variables to 0
	
	return 
	

;MAIN LOOP
main_loop:
    
    nop
    goto    main_loop
    END

