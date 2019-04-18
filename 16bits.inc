; Make T1 = T2 + T3
      
ADD16 MACRO T1, T2, T3
      MOVF   T3 + 1, w               ;  Process the High Byte First
      ADDWF  T2 + 1, w               ;   -> Can Also be "subwf"
      MOVWF  T1 + 1                  ;  Store the Result
      MOVF   T3, w                   ;  Process the Low Byte
      ADDWF  T2, w                   ;   -> Can Also be "subwf"
      MOVWF  T1
      BTFSC  STATUS, C              ;  If "subwf", use "btfss"
      INCF   T1+1                  ;   Increment the High Byte
      ENDM                          ;   If "subwf", use "decf"
; Make T1 = T2 - T3
SUB16 MACRO T1, T2, T3
      MOVF   T3 + 1, w               ;  Process the High Byte First
      SUBWF  T2 + 1, w               ;   -> Can Also be "subwf"
      MOVWF  T1 + 1                  ;  Store the Result
      MOVF   T3, w                   ;  Process the Low Byte
      SUBWF  T2, w                   ;   -> Can Also be "subwf"
      MOVWF  T1
      BTFSS  STATUS, C              ;  If "subwf", use "btfss"
      DECF   T1 + 1                  ;   Increment the High Byte
      ENDM                          ;   If "subwf", use "decf"
; 16 bit addition macro with carry out.
; Operation: SRC + DST => DST
ADDSELF16 MACRO   SRC, DST    
	  MOVF    SRC, W      ; Get Low byte
	  ADDWF   DST, F	  ; Add to destination
	  MOVF    SRC+1, W    ; Get high byte
	  BTFSC   STATUS, C   ; Check for carry
	  INCFSZ  SRC+1, W    ; Add one for carry
	  ADDWF   DST+1, F	  ; Add high byte into DST
	  ENDM

; 16 bit subtraction : DST - SRC => DST
; Carry is preserved
SUBSELF16 MACRO DST, SRC
	  MOVF  SRC, W       ; Get low byte of subtrahend
	  SUBWF DST, F       ; 
	  MOVF  SRC+1, W
	  BTFSS STATUS, C
	  INCF  SRC+1, W
	  SUBWF DST+1, F
	  ENDM
; Increment 16 bits variables : VAR = VAR + 1
INC16 MACRO VAR
      INCF  VAR ; increment lower byte
      BTFSC STATUS, Z ; if zero flag is set
      INCF VAR+1 ; skip over Upper Byte Decrement
      ENDM
; Decrement 16 variables : VAR = VAR -1
DEC16 MACRO VAR  
      MOVF VAR, F   ; Set zero flag based on low byte
      BTFSC STATUS, Z ; if low byte == 0 before
      DECF VAR+1  ; Decrement High byte
      DECF VAR
      ENDM
; 16 Bits Compare with Variable 
; The following code, while clunky will set the carry flag if "First" is equal to or greater than "Second":
COMP16 MACRO  FIRST, SECOND
       MOVF   SECOND, W             ;  Compare the High Byte First
       SUBWF  FIRST, W
       BTFSS  STATUS, Z              ;  If Result of High Byte Compare
       GOTO   Skip                   ;   is Equal to Zero, Then Check
       MOVF   SECOND+1, W          ;   the Second
       SUBWF  FIRST+1, W
       Skip:                            ;  Carry Flag Set if 1st >= 2nd
       ENDM