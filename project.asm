;*********************************( Object-Counting System )*******************************************
; * Program name: Object-Counting System
;***********************( Program description )*******************
; *
; * This program The code implements an object counting system 
; * using two sensors (t1 and t2) interfaced with the PIC16F877A microcontroller.
; * The system counts objects based on their motion direction (forward or reverse) and updates 
; * the count value on two seven-segment displays. A buzzer is activated for 0.5 seconds 
; * whenever the count value is updated. Additionally, the system checks if the count value exceeds 
; * the predefined maximum (Cmax) or falls below the predefined minimum (Cmin) and triggers 
; * LED flashing and prolonged buzzer activation for T seconds in such cases.
; * The system includes a PAUSE switch that can be used to stop the counting operation.
;*********************(System Components)*************
; * The system consists of the following components:
; * 1) A PIC16F877A microcontroller
; * 2) Two infrared sensors
; * 3) Two seven-segment displays
; * 4) An LED
; * 5) A buzzer
;***********************(Created by)*******************
; * Created by Embedded lab engineers :
; * Salah alratrout , Obada omar , Abdullah ayman , Ayamn qarout
; * Date Created: May 15th, 2023 
; * Date Last Revised: May 25th, 2023 
;**********************(Outputs & Inputs)**************
; * Outputs:
;*
; * Two seven-segment displays: These displays show the count value in (tens digit and units digit).
; * Buzzer: This device produces sound signals .
; * LED: This light-emitting diode is activated to indicate certain conditions.
; *
; * Inputs:
; *
; * Sensors t1 and t2: These sensors are used to detect the presence of objects..
; * PAUSE switch: This switch allows pausing the counting operation.

;************************** ( Header ) *****************************************************
; * the header file "p16f877a.inc" which contains definitions for all of the registers 
; * and constants used in the PIC16F877A microcontroller.
;*******************************************************************************************
#include p16f877a.inc

cblock 0x21
	TEMP
	t0Counter
	t2Counter
	PAUSE
	TENS
	UNITS
	FLAG	
	COUNTER
	VALID
	Cmin
	Cmax
	RESULT
	DELAY_COUNT
	DELAY_Inner_COUNT
	WREG_CONTEXT
	STATUS_CONTEXT
endc

org 0x00
 	goto Main
org 0x04
	goto ISR

;********************* ( MAIN ) *****************************************************
; *	The Main section is the starting point of the program.
; *	It calls the INITIAL subroutine to initialize variables and configure the ports.
; *	Then, it enters an infinite loop where it checks the state of the PAUSE switch.
; *	If the switch is open, it calls the DISPLAY subroutine 
; * and continues to the next iteration. 
; *	If the switch is closed, it waits for the switch to open again.
;************************************************************************************
Main
	call INITIAL
	goto	START

START
	CALL	DISPLAY
	BTFSS	PORTB, 4
	GOTO	SENSOR_CHECK
	GOTO	START

;************************************** ( INITIAL ) ***************************************
;The INITIAL subroutine is responsible for initializing variables and configuring the ports. 
;It sets all variables and ports to their initial values,
;configures the I/O direction for the required pins,
;sets the design parameters Cmin and Cmax,and enables interrupts.
;*******************************************************************************************
INITIAL

		;*****Configure Port*****
	
		BANKSEL 	TRISA								
		CLRF		TRISD			
		movlw		b'00010011'
		movwf		TRISB	
		CLRF		TRISA
		movlw	0x06
		movwf	ADCON1

		banksel Cmin
		MOVLW	.0
		MOVWF	Cmin
		MOVLW	.50
		MOVWF	Cmax

		;*****Clear Variable*****

		BANKSEL		TEMP   
		CLRF		TEMP
		CLRF		TENS
		CLRF		UNITS
		CLRF		COUNTER
		CLRF 		VALID
		CLRF		RESULT
		CLRF		PORTD
		CLRF		PORTB	
		CLRF		PORTC
		CLRF		PORTA

		;*****Enable Interrup & Clear Flag*****

		BANKSEL		INTCON			
		MOVLW		B'11000000'		
		MOVWF		INTCON		
		BCF			PIR1, 1			
		BANKSEL 	INTCON
		bcf			ADCON0, ADON

		RETURN

;*********************************** ( SensorCheck ) **************************************
; * The SensorCheck subroutine is called when an object is detected by the sensors. 
; * It checks the state of the sensors (t1 and t2) and waits for the sensor status to change. 
; * Depending on the sensor sequence, 
; * it either calls the INCREMENT or DECREMENT subroutine to update the count value.
;*******************************************************************************************
SENSOR_CHECK
	BTFSS	PORTB,1
	goto 	INC	
	BTFSS	PORTB,0
	GOTO	DEC
	goto 	START

;******************************** ( INCREMENT & DECREMENT ) ******************************
; * The INCREMENT and DECREMENT subroutines are responsible
; * for incrementing and decrementing the count value,respectively. 
; * They call the CHECK subroutine to validate the count value against Cmax and Cmin. 
; * If the count value is valid,
; * it jumps to either the Norm_case or Edge_case subroutine based on the validity flag.
;*******************************************************************************************
DEC
	CALL 	DISPLAY		
	BTFSC	PORTB ,1
	goto 	DEC
	CALL	CHECK_CMIN
	BTFSC	VALID,0
	GOTO	Edge_case
	DECF	COUNTER,F
	GOTO	Norm_case

INC	
		CALL	DISPLAY
		BTFSC	PORTB,0
		GOTO	INC
		CALL	CHECK_CMAX 
		BTFSC	VALID,0
		GOTO	Edge_case
		INCF	COUNTER,F
		GOTO	Norm_case

;************************* ( Norm_case & Edge_case ) ***************************************
; * The Norm_case and Edge_case subroutines handle the cases where the count value is within
; * the valid range and when it exceeds Cmax or falls below Cmin, respectively.
; * They perform the necessary actions such as initializing timers, 
; * activating the buzzer and LED, adjusting the count value if required, 
; * and calling the DISPLAY subroutine.
;*******************************************************************************************
Norm_case
		call Initial_TMR0_0.5
		CALL DISPLAY
		call ActBuzzer
		call DISPLAY
		GOTO DELAY_0.5

Edge_case
		call DISPLAY
		call Initial_TMR0_2
		call Initial_TMR2
		call DISPLAY
		call ActBuzzer
		call ActLED
		call DISPLAY	
		GOTO DELAY_2

;************************ ( CHECK ) *****************************************
; * The CHECK subroutine compares the count value with Cmax and Cmin
; * and sets the validity flag accordingly. 
; * If the count value is within the valid range, the validity flag is set,
; * otherwise,it is cleared.
;****************************************************************************
CHECK_CMAX
		CLRF	VALID
		MOVF	Cmax,W
		SUBWF	COUNTER ,W
		BTFSC	STATUS,C
		BSF 	VALID, 0
		BTFSC	STATUS, Z
		BSF 	VALID, 0
		RETURN
CHECK_CMIN
		CLRF 	VALID
		MOVF	Cmin,W
		SUBWF	COUNTER ,W
		BTFSS	STATUS,C
		BSF 	VALID, 0
		BTFSC	STATUS, Z
		BSF 	VALID, 0
		return 

;************************Convert subroutine****************************
; Subroutine to convert the 8-value in Temo to 3 BCD digits
;*********************************************************************** 

CHANGE_To_BCD
		BANKSEL		COUNTER
		movf		COUNTER, W
		movwf 		RESULT
gen_tens
  		MOVLW     	.10             
  		SUBWF     	RESULT, W          
  		BTFSS     	STATUS, C      
 		GOTO      	gen_ones        
  		MOVWF     	RESULT            
  		INCF      	TENS, F        
  		GOTO      	gen_tens        
gen_ones
  		MOVF      	RESULT, W
  		MOVWF     	UNITS           
 	 	RETURN
;**************************Display subroutine**************************
; Subroutine to show the three values on the 7-seg displays
;**********************************************************************
		
DISPLAY	
		;*****DISPLAY TENS BIT*****
		
		call 		CHANGE_To_BCD
  		MOVF     	TENS, W                  
  		CALL     	TABLE
  		MOVWF    	PORTD
 		BCF      	PORTB, 3				;ENABLE 7-SEGMENT -> HIGH DIGIT 
  		CALL     	DELAY
  		BSF      	PORTB, 3				;DISABLE 7-SEGMENT -> HIGH DIGIT 
		CLRF		TENS

  		;*****DISPLAY UNITS BIT*****

  		MOVF     	UNITS,W 
  		CALL     	TABLE
  		MOVWF    	PORTD
  		BCF      	PORTB, 2				;ENABLE 7-SEGMENT -> LOW DIGIT
  		CALL     	DELAY
  		BSF      	PORTB, 2				;DISABLE 7-SEGMENT -> LOW DIGIT
		CLRF		TENS
		CLRF		UNITS
 		RETURN


;***************************7-segments Lookup Table***********************

TABLE 
		ADDWF  	 	PCL, 1
		RETLW		B'00111111'		;'0'
		RETLW		B'00000110'		;'1'	
		RETLW		B'01011011'		;'2' 		
		RETLW		B'01001111'		;'3' 		
		RETLW		B'01100110'		;'4'
		RETLW		B'01101101'		;'5'		
		RETLW		B'01111101' 	;'6'
		RETLW		B'00000111'		;'7'		
		RETLW		B'01111111'		;'8'
		RETLW		B'01101111'		;'9'


;***************************Delay subroutines*****************************
; * The DELAY subroutine introduces a delay of approximately 
; * 0.5 seconds using a loop counter.
;*************************************************************************

DELAY
  		MOVLW    	0x0E
  		MOVWF    	TEMP
L5 		DECFSZ   	TEMP,1
  		GOTO     	L5
  		RETUrn

;*****************DELAY_0.5*******************

DELAY_0.5

	MOVLW 0x3A
 	MOVWF DELAY_COUNT
	
DELAY_LOOP 

    CALL DELAY_1MS
    DECFSZ DELAY_COUNT, F
    GOTO DELAY_LOOP
	goto START

;*****************DELAY_1MS*******************
DELAY_1MS

    MOVLW 0x3B
    MOVWF DELAY_Inner_COUNT

DELAY_1MS_LOOP
    NOP           
    NOP
    NOP
	CALL	DISPLAY 
    DECFSZ DELAY_Inner_COUNT, F
    GOTO DELAY_1MS_LOOP

    RETURN

;*****************DELAY_2*******************
DELAY_2

	MOVLW 0x7A
 	MOVWF DELAY_COUNT
	
DELAY_2_LOOP 

    CALL DELAY_2sec
    DECFSZ DELAY_COUNT, F
    GOTO DELAY_2_LOOP
	goto START
;*****************DELAY_2sec*******************
DELAY_2sec

    MOVLW 0x7A
    MOVWF DELAY_COUNT

DELAY_2sec_LOOP
    NOP           
    NOP
    NOP
	CALL	DISPLAY 
    DECFSZ DELAY_Inner_COUNT, F
    GOTO DELAY_2sec_LOOP

    RETURN
;*************************** ( Active & deActive ) ****************************************		
; * The ActBuzzer and ActLED subroutines are responsible for activating the buzzer and LED,
; * respectively. 
; * They set the corresponding pins to HIGH.
; * The deActBuzzer and deActLED subroutines are responsible for deactivating 
; * the buzzer and LED, respectively. 
; * They set the corresponding pins to LOW.
;*******************************************************************************************
ActBuzzer
		bsf	PORTA, 5
		return
	
ActLED 
		bsf	PORTA, 3
		return
	
deActBuzzer
		bcf	PORTA, 5
		return

deActLED 
		bcf	PORTA, 3
		return


;**************************Initial_TMR0_0.5 and Initial_TMR0_2*****************************

; * The Initial_TMR0_0.5 and Initial_TMR0_2 subroutines initialize 
; * the TMR0 module with different configurations to generate 
; * interrupts at specific time intervals. 
; * They configure the OPTION_REG and TMR0 registers accordingly.
;*******************************************************************************************


;*************Initial_TMR0_0.5**************
Initial_TMR0_0.5

	banksel OPTION_REG
	movlw 0xC4
	movwf OPTION_REG	
	
	banksel TMR0
	bcf	INTCON, TMR0IF
	bsf	INTCON, TMR0IE
	movlw .131
	movwf TMR0
	
	movlw .125
	movwf t0Counter
	return

;*************Initial_TMR0_2**************
Initial_TMR0_2

	banksel OPTION_REG
	movlw 0xC5
	movwf OPTION_REG	
	
	banksel TMR0
	bcf	INTCON, TMR0IF
	bsf	INTCON, TMR0IE
	movlw .131
	movwf TMR0
	
	movlw .250
	movwf t0Counter
	return
	
;*************Initial_TMR2**************
Initial_TMR2

	banksel PIE1	
	bsf PIE1, TMR2IE
	movlw .124
	movwf PR2

	banksel T2CON
	movlw b'00111110'
	movwf T2CON

	movlw .125
	movwf t2Counter
	return

;*********************************ISR subroutine*******************************
; * The ISR (Interrupt Service Routine) is responsible for handling interrupts.
; * It checks the interrupt flags (RBIF, TMR0IF, TMR2IF) and jumps to 
; * the corresponding interrupt handling routines (RB_INT, TMR0_INT, TMR2_INT) 
; * based on the flag that is set.
;******************************************************************************
ISR
	movwf WREG_CONTEXT
	btfsc INTCON, TMR0IF
	goto TMR0_INT
	btfsc PIR1, TMR2IF
	goto TMR2_INT
	retfie

;********TMR0_INT*********
TMR0_INT
	bcf INTCON, TMR0IF
	movlw .131
	movwf TMR0
	movf  WREG_CONTEXT, W
	decfsz t0Counter, F
	retfie
	bcf	INTCON, TMR0IE
	call deActBuzzer
	retfie
;********TMR2_INT*********
TMR2_INT
	banksel	PIR1
	bcf PIR1, TMR2IF
	decfsz t2Counter, F
	retfie

	banksel	PIE1
	bcf	PIE1, TMR2IE
	banksel	INTCON
	call deActLED

	retfie
;********************************************************************************

	end
	










