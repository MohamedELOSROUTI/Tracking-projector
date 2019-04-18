


; ************************************************************;
;							      ;
;	    Feedback Control of the buck converter	      ;    
;		Kp = 0.5, Ki = 0.0625, Kd = 2    	      ;
;	    Oh... des multiples de deux, on se demande pq ... ;
;							      ;
; ************************************************************;

    processor 16f1789
    #include "config.inc"
    #include "16bits.inc"
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Macro's ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
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
    
    ADJUST_dutyPWM macro
	movlb 0x01
	COMP16 ADRESL, ref
	btfss STATUS, C ; if C = 1 ADRES>= ref
	goto INCREASE_DUTY
	movlb 0x05
	DEC16 CCPR2L
	retfie
    INCREASE_DUTY:
	movlb 0x05
        INC16 CCPR2L
	retfie
	endm
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; END OF MACRO's ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
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
    RESET_TMR1_SAMPLING
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
    ADC_OFF
    ADJUST_dutyPWM ; with saturation
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
	movlw b'01000011' ; initialise the duty cycle to 0.5 => counter = b'100001100'
	movwf CCPR2L
	
	; Setup TMR1 for sampling process
	
	movlb 0x00
	movlw b'11111011' ; TMR1 = 65268 in order to have overflow after 267 ins
	movwf TMR1H       ; The overflow frequency corresponds to 15 Khz for the sampling frequency : T = 2*Ts
	movlw b'11010100'
	movwf TMR1L      
	movlw b'00000001' ; Prescaler 1:1 + enable timer 1
	movwf T1CON
	
	; Interrupt declaration of timer 1 + Enabling ADC interrupt
	movlb 0x01
	movlw b'01000001' ; Enables TMR1 overflow interrupt and ADC interrupt
	movwf PIE1
	bsf INTCON, 7 ; Enables all active interrupts
	bsf INTCON, 6 ; Enables all active peripheral interrupts
	; General Purpose ram Bank 0 for storing current PWM value of 2 bytes
	; Variables declaration for PID computation : Common Ram location at 70h
	cblock 70h
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

	return 
	
main_loop:
    nop
    goto main_loop
    END
    
    ;; Prochaine �tape : d�buggage du code :D
    ;; mettre une valeur dans adresh:addreshl pour simuler vout
    ;; regarder la valeur du dutyPWM (voir si elle varie bien dans le bon sens)
    ;; A la place du dutyPWM on peut voir le registre CCPR2L : 8 bits poids for pwm
    ;; ensuite mettre vout � vref et verifie que dutyPWM ne varie plus (ou tres tres peu pour ne plus varier arp�s)