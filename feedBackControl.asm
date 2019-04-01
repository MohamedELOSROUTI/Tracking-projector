; ************************************************************;
;							      ;
;	    Feedback Control of the buck converter	      ;    
;		Kp = 0.5, Ki = 0.0625, Kd = 2    	      ;
;	    Oh... des multiples de deux, on se demande pq ... ;
;							      ;
; ************************************************************;

    processor 16f1789
    #include "config.inc"
    ; Macros 
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
	movlw b'11110100' ; 1 inst
	movwf TMR1L ; 1 instr
	movlw b'11111110' ; 1 instr
	movwf TMR1H ; 1 instr
	bsf   T1CON, 0 ; activate timer 1
	endm
	
    RESET_COUNTER macro
 
	movlw b'01000000'
	movwf counter
	endm
	
    COMPUTE_ERROR_CURRENT macro
 
	movf ADRESH, 0
	subwf ref+1,0
	movwf errCurrent+1
	movf ADRESL,0
	subwf ref, 0
	movwf errCurrent
	btfss STATUS, C
	decf errCurrent+1,1
	endm
	
    COMPUTE_up macro ; up = errCurrent * Kp, with Kp = 2
 
	rlf errCurrent+1,0
	movwf up+1
	rlf errCurrent, 0
	movwf up
	btfsc STATUS, C
	bsf up+1,0
	endm
	
    COMPUTE_ui macro ; ui = errCurrent*Ki+ui, with Ki = 2^-4
		     ; use ud as tmp variable :D (not enough common RAM) 
	; ud = errCurrent*Ki
	rrf errCurrent, 0
	movwf ud
	rrf ud, 1
	rrf ud, 1
	rrf ud, 1
	rlf errCurrent+1, 0
	movwf ud+1
	rlf ud+1, 1
	rlf ud+1, 1
	rlf ud+1, 1
	movf ud+1, 0
	addwf ud, 1
	rrf errCurrent+1, 0
	movwf ud+1
	rrf ud+1,1
	rrf ud+1,1
	rrf ud+1,1
	
	; ui=ud+ui
	movf ud, 0
	addwf ui, 1
	btfsc STATUS, C
	incf ui+1,1
	movf ud+1,0
	addwf ui+1,1
	endm
	
    COMPUTE_ud macro ; ud = Kd*(errCurrent-errPrevious)
	; compute first ud = errCurrent-errPrevious
	movf errPrevious+1, 0
	subwf errCurrent+1, 0
	movwf ud+1
	movf errPrevious, 0
	subwf errCurrent, 0
	movwf ud
	btfss STATUS, C
	decf ud+1,1
	
	; Now make ud <- 2ud
	rlf ud+1, 1
	rlf ud, 1
	btfsc STATUS, C
	incf ud+1, 1
	endm
	
    COMPUTE_upid macro ; upid = up+ui+ud
	; First upid <- up + ui
	movf ui+1,0
	addwf up+1,0
	movwf upid+1
	movf ui,0
	addwf up,0
	movwf upid
	btfsc STATUS, C
	incf upid+1, 1
	; Second upid <- upid + ud
	movf ud, 0
	addwf upid, 1
	btfsc STATUS, C
	incf upid+1,1
	movf ud+1,0
	addwf upid+1,1
	endm
    UPDATE_ERROR_PREVIOUS macro
	movf errCurrent, 0
	movwf errPrevious
	movf errCurrent+1, 0
	movwf errPrevious+1
	endm
    ADJUST_upid macro ; make it between [0,2^10-1]
	movlw b'00001000'
	addwf upid+1, 1 ; now upid is in [0, 2^12-1]
	; make it between [0,2^10-1] for use with pwm
	rlf upid+1,1
	rlf upid+1,1
	rlf upid+1,1
	rlf upid+1,1
	movlw b'11110000'
	andwf upid, 0
	movwf ud ; again we use register ud here 
	rrf ud,1
	rrf ud,1
	rrf ud,1
	rrf ud,1
	movf ud, 0
	addwf upid+1,1
	rlf upid,1
	rlf upid,1
	movlw b'00110000'
	andwf upid, 1
	endm
    
    CONFIGURE_PWM macro
	movlb 0x05
	movf upid+1,0
	movwf CCPR2L ; MSB
	movlw b'00001111'
	andwf CCP2CON, 1
	movf upid, 0
	addwf CCP2CON, 1
	endm
    ; Start execution of the program by the microcontroller here
    org 0x00
    nop
    goto start
    
    ; Interrupt routine 
    org 0x04
    movlb 0x00
    btfss PIR1, 0 ; test if interrupt flag of timer 1 is set
    goto ADC_interrupt
    CLEAR_FLAG_TMR1
    RESET_COUNTER
    ADC_ON
WaitAquisition:
    decf counter, 1
    btfss STATUS, 2 ; test Zero bit : if counter == 0 skip next instruction
    goto WaitAquisition 
    bsf   ADCON0, 1 ; Start an ADC conversion cycle
Conversion:
    btfsc ADCON0, 1 ; is conversion done ?
    goto  Conversion
ADC_interrupt:
    movlb 0x00
    btfss PIR1, 6
    retfie
    CLEAR_FLAG_ADC
    COMPUTE_ERROR_CURRENT
    ADC_OFF
    COMPUTE_up
    COMPUTE_ui
    COMPUTE_ud
    COMPUTE_upid ; upid is a variable between [-1925, 273] ; need to be adjusted (positive+2^11) -> [0, 2^12-1]-> [0, 2^10-1] (for pwm)
    UPDATE_ERROR_PREVIOUS ; errPrevious = errCurrent
    ADJUST_upid ; upid is adjusted now : between  [0, 2^10-1] but all shifted to the left in order to equalize it easily to pwm registers
    CONFIGURE_PWM ; adjust PWM according to upid
    RESET_TMR1_SAMPLING
    retfie

    ; Beginning of the program here
start:
	call initialisation 
	goto main_loop
	
initialisation:
	; Configure port for PWM output : PORT C
	movlb 0x01
	bcf TRISC, 1 ; make port RC1 (CCP2 pin) as output
	bcf LATC, 1 ; initialise output RC1 = 0
	
	; Configure port for analog to digital conversion : PORT D
	movlb 0x01
	bsf TRISD, 1 ; make port RD1 as input 
	movlb 0x03
	bsf ANSELD, 1 ; make port RD1 as analog input
	movlb 0x04
	bcf WPUD, 1 ; disable weak pull-up on RD1
		
	; Configuration of system clock 
	movlb 0x01
	movlw b'01111010' ; Configure internal clock to 16 Mhz (in order to get the best PWM resolution : here 9 bits ) 
	movwf OSCCON
	movlw b'00000000' ; Oscillator module must run at the facory calibrated frequency
	movwf OSCTUNE
	
	; ADC parameters
	
	movlb 0x01
	movlw b'11010100' ; use channel 21 + 10 bits conversion + 2's representation :D
	movwf ADCON0
	movlw b'01000000' ; Vref+ and Vref- to VDD, VSS, ADC conversion clock = Fosc/4, sign magnitude result
	movwf ADCON1
	
	; Configure TMR2 for use with PWM
	movlb 0x00
	bcf PIR1, 1 ; clear TMR2 interrupt flag 
	movwf b'00000100' ; prescalar = 1, postscalar = 1:1, set timer2 on
	movwf T2CON
	
	; Setup PWM 
	movlb 0x00
	movlw b'10000101'; set PR2 = 133 : resolution of 9 bits and Ts = 30 kHz
	movwf PR2
	movlb 0x05
	movlw b'00001100' ; set PWM mode and the PWM duty cycle LSB to '00'
	movwf CCP2CON
	movlw b'00000000' ; initialise the duty cycle to 0
	movwf CCPR2L
	
	; Setup TMR1 for sampling process
	
	movlb 0x00
	movlw b'11111110' ; TMR1 = 65268 in order to have overflow after 267 ins
	movwf TMR1H       ; The overflow frequency corresponds to 15 Khz for the sampling frequency : T = 2*Ts
	movlw b'11110100'
	movwf TMR1L      
	movlw b'00000001' ; Prescaler 1:1 + enable timer 1
	movwf T1CON
	
	; Interrupt declaration of timer 1 + Enabling ADC interrupt
	movlb 0x01
	movlw b'01000001' ; Enables TMR1 overflow interrupt and ADC interrupt
	movwf PIE1
	bsf INTCON, 7 ; Enables all active interrupts
	bsf INTCON, 6 ; Enables all active peripheral interrupts
	
	; Variables declaration for PID computation : Common Ram location at 70h
	cblock 70h
	errCurrent   : 2 ; 2 bytes for the error representation (err = ref-vout)
	errPrevious  : 2 ; 2's representation of the error term 
	up           : 2 ; 2 bytes is (largely) enough for the proportionnal term
	ui           : 2 ; 2 bytes is enough (largely) for the integrative term
	ud           : 2 ; 2 bytes is enough for the derivative term
	upid         : 2 ; 2 bytes is needed for upid=ui+up+ud
	ref          : 2 ; 2 bytes is enough for ref : = 273 = 111h = b'0000000100010001'
	counter      : 1 ; 1 byte : initialize to 64 : use for acquisition time ADC
	endc
	
	; Set the reference to 442h
	movlw b'00010001'
	movwf ref ; Lower byte
	movlw b'00000001'
	movwf ref+1 ; Upper byte
	
	; Initialize counter 
	movlw b'01000000' ; 64
	movwf counter
	; Initialize all variables to 0
	movlw b'00000000'
	andwf errCurrent,1
	andwf errCurrent+1,1
	andwf up,1
	andwf up+1,1
	andwf ui,1
	andwf ui+1,1
	andwf ud,1
	andwf ud+1,1
	andwf upid, 1
	andwf upid+1,1
	
	
main_loop:
    nop
    goto main_loop
    END
    
    
	