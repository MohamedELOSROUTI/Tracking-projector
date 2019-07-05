; RD0 : output pin signal
; RD1 : input pin 
; send signal (1111)010110101110
    
processor 16f1789

#include "p16f1789.inc"

; CONFIG1
; __config 0x2F82
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_ON & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_ON
; CONFIG2
; __config 0x1EFF
 __CONFIG _CONFIG2, _WRT_OFF & _VCAPEN_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

 RD0_ON macro
 banksel LATD
 bsf LATD, 0
 endm
 
 RD0_OFF macro
 baksel LATD
 bcf LATD, 0
 endm
 
 org 0x00
 nop 
 goto start
 
 ; Interrupt routine
 org 0x04
 btfss PIR1, TMR1IF
 retfie
 bcf PIR1, TMR1IF
 
 
 movf counter, F
 btfss STATUS, Z
 goto one
 RD1_ON
 incf counter, F
 retfie
 
 one:
 movf counter, W
 sublw d'1'
 btfss STATUS, Z
 goto two
 RD1_ON
 incf counter, F
 retfie

 two:
 movf counter, W
 sublw d'2'
 btfss STATUS, Z
 goto three
 RD1_ON
 incf counter, F
 retfie
 
 three:
 movf counter, W
 sublw d'3'
 btfss STATUS, Z
 goto four
 RD1_ON
 incf counter, F
 retfie
 
 four:
 movf counter, W
 sublw d'4'
 btfss STATUS, Z
 goto five
 RD1_OFF
 incf counter, F
 retfie
 
 five:
 movf counter, W
 sublw d'5'
 btfss STATUS, Z
 goto six
 RD1_ON
 incf counter, F
 retfie
 
 six:
 movf counter, W
 sublw d'6'
 btfss STATUS, Z
 goto seven
 RD1_OFF
 incf counter, F
 retfie
 
 seven:
 movf counter, W
 sublw d'7'
 btfss STATUS, Z
 goto eight
 RD1_ON
 incf counter, F
 retfie
 
 eight:
 movf counter, W
 sublw d'8'
 btfss STATUS, Z
 goto nine
 RD1_ON
 incf counter, F
 retfie
 
 nine:
 movf counter, W
 sublw d'9'
 btfss STATUS, Z
 goto ten
 RD1_OFF
 incf counter, F
 retfie
 
 ten:
 movf counter, W
 sublw d'10'
 btfss STATUS, Z
 goto eleven
 RD1_ON
 incf counter, F
 retfie
 
 eleven:
 movf counter, W
 sublw d'11'
 btfss STATUS, Z
 goto twelve:
 RD1_OFF
 incf counter, F
 retfie
 
 twelve:
 movf counter, W
 sublw d'12'
 btfss STATUS, Z
 goto thirteen
 RD1_ON
 incf counter, F
 retfie
 
 thirteen:
 movf counter, W
 sublw d'13'
 btfss STATUS, Z
 goto forteen
 RD1_ON
 incf counter, F
 retfie
 
 forteen:
 movf counter, W
 sublw d'14'
 btfss STATUS, Z
 goto fifteen
 RD1_ON
 incf counter, F
 retfie
 
 fifteen:
 movf counter, W
 sublw d'15'
 btfss STATUS, Z
 RD1_OFF
 clrf counter
 retfie
 
 
; Beginning of the program here
 
 start:
    call initilialisation
    goto main_loop
   
initialisation:

    cblock 70h
    counter
    endc
    clrf counter
    return
    
main_loop:
    
init_pin:
    
    
goto main_loop
END