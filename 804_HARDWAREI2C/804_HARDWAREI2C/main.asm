          ;
; 804_HARDWAREI2C.asm
;
; Created: 2/1/2021 11:35:30 PM
; Author : pappan
;
;max 204,min420 for 55l . .25l per adc value

;PB1 SDA = 8
;PB0 SCL = 9
;PA1 - FREQUENCY INPUT =11
;PA2 - ADC FOR TEMPRATURE =12 =AIN2
;PA3 - ADC FOR FUEL LEVEL =13 =AIN3
 ;T = ((1024-V1)/10)+33

.equ data_command1 = 0b00001001		; data control nibble ,led on P3, EN 0 on P2, R/W 0 (write) in P1 , RS 1 (0 instruction, 1 data) = 1001  =0x09
.equ data_command2 = 0b00001101		; data control nibble , 1101  = 0x0D   - EN goes hi=1
.equ data_command3 = 0b00001001		; data control nibble , 1001  = 0x09   - EN goes low=0
.equ inst_command1 = 0b00001000		;instruction control nibble ,  led on en-lo,Rw-0,rs =0   = 1000   = 0x08
.equ inst_command2 = 0b00001100		;instruction control nibble ,  led on,EN hi , rs/RW 0    = 1100   = 0x0C
.equ inst_command3 = 0b00001000		;instruction control nibble  , led on, EN lo ,rs/rw 0    = 1000   = 0x08
;.equ slave_address = 0b01001110		;0x4E 
.EQU SLAVE_ADDRESSW = 0X4E   ; OLED = 0X78 ,1602 =4E
.equ fclk = 10000000
.DEF  SLAVE_REG = R17
.DEF  TEMP = R16
.DEF  HundredCounter = R20
.DEF  TenthCounter	= R21				
.DEF  OneCounter = R22				
.DEF  ZEROREG = R23

.macro ASCII_SETUP
	LDS R17,@0						; LOAD R17 WITH ADC_H
	LDS R16,@1
.endm
	
.DSEG
BUFFER: .BYTE 8
TEMP_BUFFERL: .BYTE 1
TEMP_BUFFERH: .BYTE 1
FUEL_BUFFERL: .BYTE 1
FUEL_BUFFERH: .BYTE 1
PAD: .BYTE 1


.CSEG
.ORG 0X00
rjmp reset
;.ORG 0X25
;rjmp TWI0_TWIM_vect_isr


;TWI0_TWIM_vect_isr:
;RETI


reset:
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PROTECTED WRITE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ldi r16,0b01111110  ; osclock =0 bit7  and bit 1:0 = 0x2 for 20mhz ,all reserved bits to ber witten as 1.
;out OSCCFG,r16

	ldi r16,0Xd8
	out CPU_CCP,r16
	ldi r16,0x01
	STS CLKCTRL_MCLKCTRLB,R16
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;writing strings to LCD
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	ldi ZH,high(2*string)			;initialize Z pointer 
	ldi ZL,low(2*string)			;initialize Z pointer 
	ldi XH,high(BUFFER)				;initialize X pointer 
	ldi XL,low(BUFFER)				;initialize X pointer 
rloop:								;LOADS STRING TO BUFFER IN  SRAM
	lpm r16,Z+						;copy array from flash
	st X+,r16						;store array in memory/buffer pointed by X
	inc r17							;holds string array length , 5 for HELLO
	cpi r17,5
	brne rloop

		
		RCALL LCD_INIT				;initialize LCD ,cursor in 1st line 1 column after initialization
		ldi XH,high(BUFFER)			;initialize X pointer
		ldi XL,low(BUFFER)			;initialize X pointer
WLOOP:	LDI SLAVE_REG,SLAVE_ADDRESSW	;load r17 with I2C LCD address
		RCALL TWI_INIT				;send I2C start & address
		LD R17,X+					;copy data pointed by X from array to r17/SLAVE_REG (HELLO)
		CPI R17 ,0X00				;check if the data is null because null is taken as the terminator of copy
		BREQ NULL					;exit address of loop on detecting null
		RCALL DATA_WRITE			;call DATA_WRITE subroutine
		ldi temp,20
		rcall delayTx1uS
		RJMP WLOOP					;loop back until null is detected
NULL:	
		LDI YH,HIGH(2000)			;load delay value = 2000ms
		LDI YL,LOW(2000)			;load delay value = 2000ms
		RCALL delayYx1mS			;call ms delay routine for 2 seconds
		LDI SLAVE_REG,SLAVE_ADDRESSW	;load r17 with I2C LCD address
		RCALL TWI_INIT				;send I2C start & address
		LDI SLAVE_REG,0b00000001	;load SLAVE_REG/r17 with clear screen command of LCD
		RCALL COMMAND_WRITE			;call COMMAND_WRITE subroutine
		ldi temp,20
		rcall delayTx1uS
		
		ldi ZH,high(2*string1)		;initialize Z pointer to string1 address
		ldi ZL,low(2*string1)		;initialize Z pointer to string1 address
SLOOP1:	LDI SLAVE_REG,SLAVE_ADDRESSW	;load r17 with I2C LCD address
		RCALL TWI_INIT				;send I2C start & address
		LD R17,Z+					;copy data pointed by Z from array to r17/SLAVE_REG
		CPI R17 ,0X00				;check if the data is null because null is taken as the terminator of copy
		BREQ NULL1					;exit address of loop on detecting null
		RCALL DATA_WRITE			;call DATA_WRITE subroutine
		ldi temp,20
		rcall delayTx1uS
		RJMP SLOOP1					;loop back until null is detected
NULL1:	
		LDI SLAVE_REG,SLAVE_ADDRESSW	;load r17 with I2C LCD address
		RCALL TWI_INIT				;send I2C start & address
		LDI SLAVE_REG,0b11000000	;(0xC0) second line,first digit  FOR TEMP
		RCALL COMMAND_WRITE			;call COMMAND_WRITE subroutine
		ldi temp,20
		rcall delayTx1uS

		ldi ZH,high(2*string2)		;initialize Z pointer to string2 address
		ldi ZL,low(2*string2)		;initialize Z pointer to string2 address
SLOOP2:	LDI SLAVE_REG,SLAVE_ADDRESSW	;load r17 with I2C LCD address
		RCALL TWI_INIT				;send I2C start & address
		LD R17,Z+					;copy data pointed by Z from array to r17/SLAVE_REG
		CPI R17 ,0X00				;check if the data is null because null is taken as the terminator of copy
		BREQ NULL2					;exit address of loop on detecting null
		RCALL DATA_WRITE			;call DATA_WRITE subroutine
		ldi temp,20
		rcall delayTx1uS
		RJMP SLOOP2					;loop back until null is detected
NULL2:
		LDI SLAVE_REG,SLAVE_ADDRESSW	;load r17 with I2C LCD address	
		RCALL TWI_INIT				;send I2C start & address
		LDI SLAVE_REG,0b11001001	;SECOND LINE 10TH POS FOR FUEL  
		RCALL COMMAND_WRITE			;call COMMAND_WRITE subroutine
		ldi temp,20
		rcall delayTx1uS

		ldi ZH,high(2*string3)		;initialize Z pointer to string3 address
		ldi ZL,low(2*string3)		;initialize Z pointer to string3 address
SLOOP3:	LDI SLAVE_REG,SLAVE_ADDRESSW	;load r17 with I2C LCD address	
		RCALL TWI_INIT				;send I2C start & address
		LD R17,Z+					;copy data pointed by Z from array to r17/SLAVE_REG
		CPI R17 ,0X00				;check if the data is null because null is taken as the terminator of copy
		BREQ NULL3					;exit address of loop on detecting null
		RCALL DATA_WRITE			;call DATA_WRITE subroutine
		ldi temp,20
		rcall delayTx1uS
		RJMP SLOOP3					;loop back until null is detected
NULL3:
		LDI SLAVE_REG,SLAVE_ADDRESSW	;load r17 with I2C LCD address
		RCALL TWI_INIT				;send I2C start & address
		LDI SLAVE_REG,0b11001111	;SECOND LINE 16TH POS FOR L
		RCALL COMMAND_WRITE			;call COMMAND_WRITE subroutine
		LDI SLAVE_REG,SLAVE_ADDRESSW	;load r17 with I2C LCD address
		RCALL TWI_INIT				;send I2C start & address
		LDI SLAVE_REG,'L'			;load SLAVE_REG with ASCII character 'L'
		RCALL DATA_WRITE			;call DATA_WRITE subroutine

		



MAIN:
		RCALL ADC_INIT
		RCALL ADC_START
		RCALL ADC_READ_FLAG
		RCALL CHANGE_MUX_POS
		RCALL ADC_READ_RESULT
		RCALL ADC_READ_FLAG
		RCALL ADC_READ_RESULT
		RCALL ADC_STOP
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI SLAVE_REG,0b11001101		;SECOND LINE 14TH POS FOR fuel VALUE
		RCALL COMMAND_WRITE
		RCALL FUEL_ADC_RANGE_CHECK
		;ASCII_SETUP FUEL_BUFFERH,FUEL_BUFFERL
		RCALL FUEL_CALC
		;CPI TenthCounter,0X30
		;BREQ SUPRESS_LEAD_ZERO
		;LDI SLAVE_REG,SLAVE_ADDRESSW
		;RCALL TWI_INIT
		;MOV SLAVE_REG,HundredCounter
		;RCALL DATA_WRITE
		;LDI TEMP,50
		;RCALL delayTx1uS
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		MOV SLAVE_REG,TenthCounter
		RCALL DATA_WRITE
		LDI TEMP,50
		RCALL delayTx1uS
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		MOV SLAVE_REG,OneCounter
		RCALL DATA_WRITE
		LDI TEMP,50
		RCALL delayTx1uS
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI TEMP,50
		RCALL delayTx1uS
		RCALL TEMP_CALC
		LDI SLAVE_REG,0b1100101					;0XC5 IS 6THE PLACE SECOND LINE
		RCALL COMMAND_WRITE
		LDI TEMP,50
		RCALL delayTx1uS
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		MOV SLAVE_REG,HundredCounter
		RCALL DATA_WRITE
		LDI TEMP,50
		RCALL delayTx1uS
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		MOV SLAVE_REG,TenthCounter
		RCALL DATA_WRITE
		LDI TEMP,50
		RCALL delayTx1uS
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		MOV SLAVE_REG,OneCounter
		RCALL DATA_WRITE
		LDI TEMP,50
		RCALL delayTx1uS

SPEED_LOOP:
		LDI R25,10
		RCALL SPEED_INIT
		RCALL CAPT_FLAGS
		RCALL SPEED_STOP
		rcall SPEED_CALC
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI TEMP,50
		RCALL delayTx1uS
		LDI SLAVE_REG,0b10000110					;RETURN HOME FOR lcd, 1ST LINE 6 POSITION
		RCALL COMMAND_WRITE
		LDI TEMP,50
		RCALL delayTx1uS
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		MOV SLAVE_REG,HundredCounter
		RCALL DATA_WRITE
		LDI TEMP,50
		RCALL delayTx1uS
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		MOV SLAVE_REG,TenthCounter
		RCALL DATA_WRITE
		LDI TEMP,50
		RCALL delayTx1uS
		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		MOV SLAVE_REG,OneCounter
		RCALL DATA_WRITE
		LDI TEMP,50
		RCALL delayTx1uS
		DEC R25
		BRNE SPEED_LOOP
		RJMP MAIN








SUPRESS_LEAD_ZERO:
		LDI TenthCounter,' '
		ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C ROUTINES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

TWI_INIT:
		ldi r16,40					;load baud rate for I2C , (fclck/2*bitrate)-10 , freq = 10Mhz
		sts TWI0_MBAUD,R16			; store into baud register
		LDI R16,0b00000011			;SMEN,ENABLE
		STS TWI0_MCTRLA,R16
		LDI R16,0b00001000			;FLUSH ADDR & DATA REGISTERS
		STS TWI0_MCTRLB,R16
		LDI R16,0b00000001			;FORCE IDLE
		STS TWI0_MSTATUS,R16
		MOV R16,SLAVE_REG			; SLAVE_REG IS R17, READ OR WRITE ADDRESS SHOULD BE LOADED HERE PRIOR TO CALL INIT
		STS TWI0_MADDR,R16
		RCALL WAIT_WIF
		SBRC R16,4					;SKIP NEXT INSTRUCTION IF RXACK IS SET
		RCALL TWI_STOP
		RET

TWI_WRITE:

		MOV R16,SLAVE_REG			;mov data from SLAVE_REG/r17 to r16
		STS TWI0_MDATA,R16			;copy r16 to MDATA register for I2C transmission
		RCALL WAIT_WIF				;call WAIT_WIF subroutine to check write interrupt flag is set after writing finished
		SBRC R16,4					;skip if bit in r16 is cleared , if ack is not received #4 will be 1
		RCALL TWI_STOP				;I2C stop if ACK not received after write finished verified by WAIT_WIF
		RET

/*TWI_READ:
		;MOV R16,SLAVE_REG			; SLAVE_REG IS R17,  WRITE ADDRESS SHOULD BE LOADED HERE PRIOR TO CALL
		;STS TWI0_MADDR,R16
		;RCALL WAIT_WIF
		MOV R16,SLAVE_REG			; READ_ADDRESS IS THE INTERNAL ADDRESS OF THE SLAVE FROM WHICH DATA IS READ
		STS TWI0_MDATA,R16
		RCALL WAIT_WIF
		MOV R16,SLAVE_REG			; SLAVE_REG IS R17,  READ ADDRESS SHOULD BE LOADED HERE FOR READING DATA FROM SLAVE READ_ADDRESS GIVEN ABOVE
		STS TWI0_MADDR,R16			; THIS IS A REPEATED START
		RCALL WAIT_RIF
		LDS R16,TWI0_MDATA			;MDATA REGISTER IS COPIED TO R16,DATA IS RECIVED INTO MDATA FROM SLAVE
		STS BUFFER,R16				;DATA IN R16 IS STORED IN SRAM BUFFER FOR LATER USE. THIS PROCESS OF READ CAN BE DONE MULTIPLE TIMES TILL 1 BEFORE LAST READ
		RCALL WAIT_RIF
		LDI R16,0b00000111			;CLEAR ACKACT BIT BEFORE READING LAST BYTE AND ISSUE A STOP = NACK+STOP
		STS TWI0_MCTRLB,R16
		LDS R16,TWI0_MDATA			;MDATA REGISTER IS COPIED TO R16,THIS THE LAST DATA IS RECEIVED  FROM SLAVE
		STS BUFFER,R16				;DATA IN R16 IS STORED IN SRAM BUFFER FOR LATER USE. 
		RET*/


TWI_STOP:
		LDI R16,0b00000011                       ;STOP
		STS TWI0_MCTRLB,R16
		RET




WAIT_WIF:
		LDS R16,TWI0_MSTATUS
		SBRS R16,6					;CHECK WIF IS SET,IF SET SKIP NEXT INSTRUCTION
		RJMP WAIT_WIF
		RET


WAIT_RIF:
		LDS R16,TWI0_MSTATUS
		SBRS R16,7					;CHECK WIF IS SET,IF SET SKIP NEXT INSTRUCTION
		RJMP WAIT_RIF
		RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


/* UNLOCKING A FROZEN I2C NETWORK
1) DISABLE TWI
2)FORCE SCL LOW
3)FLOAT SDA , TRISTATE
4) MANUALLY DRIVE SCL HIGH 6US , LOW 6US 9 TIMES
5) ENABLE TWI
6)SEND STOP*/
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;LCD ROUTINES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
COMMAND_WRITE:
		STS PAD,R17					;copy SLAVE_REG to SRAM address PAD for processing
		ANDI R17,0XF0				;preserve upper nibble of SLAVE_REG by anding with 0xF0 ,lower 4 becomes 0
		ORI R17,inst_command1		;add instruction_command1 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS
		ORI R17,inst_command2		;add instruction_command2 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS
		ORI R17,inst_command3		;add instruction_command3 to lower nibble of r17 by OR ing it
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS		
		LDS R17,PAD					;copy back data from PAD to r17 for processing the remaining lower nibble
		SWAP R17					;swap the nibbles in R17 so thlower nibble will occupy the upper area of reg
		ANDI R17,0XF0				;preserve upper nibble of SLAVE_REG by anding with 0xF0
		ORI R17,inst_command1		;add instruction_command1 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS
		ORI R17,inst_command2		;add instruction_command2 to lower nibble of r17 by OR ing it
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS
		ORI R17,inst_command3		;add instruction_command3 to lower nibble of r17 by OR ing it
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		RCALL TWI_STOP				;call TWI_STOP
		RET	


DATA_WRITE:
		STS PAD,R17					;copy SLAVE_REG to SRAM address PAD for processing
		ANDI R17,0XF0				;preserve upper nibble of SLAVE_REG by anding with 0xF0,lower 4 becomes 0
		ORI R17,data_command1		;add data_command1 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS
		ORI R17,data_command2		;add data_command2 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS
		ORI R17,data_command3		;add data_command3 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS
		LDS R17,PAD					;copy back data from PAD to r17 for processing the remaining lower nibble
		SWAP R17					;swap the nibbles in R17 so thlower nibble will occupy the upper area of reg
		ANDI R17,0XF0				;preserve upper nibble of SLAVE_REG by anding with 0xF0
		ORI R17,data_command1		;add data_command1 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS
		ORI R17,data_command2		;add data_command2 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		ldi temp,20
		rcall delayTx1uS
		ORI R17,data_command3		;add data_command3 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		RCALL TWI_STOP				;call TWI_STOP
		RET	

LCD_INIT:
		LDI TEMP,50
		RCALL delayTx1mS
		LDI SLAVE_REG,SLAVE_ADDRESSW
 		RCALL TWI_INIT
		LDI R17,0b00111100
		RCALL TWI_WRITE
		ldi temp,20
		rcall delayTx1uS
		LDI R17,0B00111000
		RCALL TWI_WRITE
		RCALL TWI_STOP
		ldi temp,20							; value loaded here (20) decides the number of milli seconds in the delay below
 		rcall delayTx1mS

		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI R17,0b00111100
		RCALL TWI_WRITE
		ldi temp,20
		rcall delayTx1uS
		LDI R17,0B00111000
		RCALL TWI_WRITE
		RCALL TWI_STOP
		ldi temp,20							; value loaded here (20) decides the number of milli seconds in the delay below
 		rcall delayTx1mS

		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI R17,0b00111100
		RCALL TWI_WRITE
		ldi temp,20
		rcall delayTx1uS
		LDI R17,0B00111000
		RCALL TWI_WRITE
		RCALL TWI_STOP
		ldi temp,20							; value loaded here (20) decides the number of milli seconds in the delay below
 		rcall delayTx1mS

		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI R17,0b00101100
		RCALL TWI_WRITE
		ldi temp,20
		rcall delayTx1uS
		LDI r17,0b00101000
		RCALL TWI_WRITE
		RCALL TWI_STOP
		ldi temp,20							; value loaded here (20) decides the number of milli seconds in the delay below
 		rcall delayTx1mS

		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI R17,0b00101000
		RCALL COMMAND_WRITE
		ldi temp,20
		rcall delayTx1uS

		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI R17,0b00001110
		RCALL COMMAND_WRITE

		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI R17,0b00000110
		RCALL COMMAND_WRITE
		ldi temp,20
		rcall delayTx1uS

		LDI SLAVE_REG,SLAVE_ADDRESSW
		RCALL TWI_INIT
		LDI R17,0b00000001
		RCALL COMMAND_WRITE
		ldi temp,20
		rcall delayTx1uS
		RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ADC routine
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ADC_INIT:
	ldi r16,0b00000100				;disable digital input buffer to enable analoge read
	sts PORTA_PIN2CTRL,r16			;0x4 in bit2:0 for disabling digital input,pull up also disables
	sts PORTA_PIN3CTRL,r16			;0x4 in bit2:0 for disabling digital input
	LDI R16,0b00000110
	STS ADC0_CTRLB,R16				;64 SAMPLES ARE ACCUMULATED
	LDI R16,0b01010011
	STS ADC0_CTRLC,R16				;SAMCAP=1,REFSEL = 0X1 VDD,PRESCALE =0X3 16
	LDI R16,0b00000010              ;0b00000010 is bit mask for AIN2 ,0b00000011 is bit mask for AIN3
	STS ADC0_MUXPOS,R16				;0X02 STORED IN UXPOS FOR AIN2 AS INPUT
	LDI R16,0B00000001
	STS ADC0_INTCTRL,R16			;ENABLE RESULT READY INTERRUPT,NEED TO PROVIDE ISR
	STS ADC0_INTFLAGS,R16			;CLEAR ANY FLAGS IN FLAG REGISTER
	RET

ADC_START:
	LDS R16,ADC0_CTRLA
	ORI R16,0b00000001				; bit mask for enabling ADC
	STS ADC0_CTRLA,R16				;ENABLE ADC
	LDI R16,0X01
	STS ADC0_COMMAND,R16			;STARTS CONVERSION FOR BOTH SINGLE & FREERUNNING MODE
	RET


ADC_STOP:
	LDS R16,ADC0_CTRLA
	ORI R16,0b00000000
	STS ADC0_CTRLA,R16				;ENABLE ADC
	LDI R16,0X00
	STS ADC0_COMMAND,R16			;STARTS CONVERSION FOR BOTH SINGLE & FREERUNNING MODE
	RET

ADC_READ_FLAG:
	LDS R16,ADC0_INTFLAGS			;THE FLAG IS SET IF RESULT IS READY,AUTOCLEAR IF READ RESULT REGISTER OR WRITING 1 TO REG
	ANDI R16,0b00000001
	BREQ ADC_READ_FLAG
	RET

ADC_READ_RESULT:
	LDS R17,ADC0_RESH
	LDS R16,ADC0_RESL
ADC_DIVIDE_64:   					;USE MACRO - ASCII_SETUP @0,@1 TO CHOOSE BUFFERS TO BE LOADED BEFORE CALLING DIVIDE_64 SUBROUTINE
	LDI R18,6						; LOAD COUNTER VALUE 6 (6 SHIFTS FOR DIVIDE WITH 64)
	LSR R17							; LOGICAL SHIFT RIGHT
	ROR R16							; ROTATE THROUGH CARRY LOW REGISTER
	DEC R18							; DECREASE COUNTER VALUE
	BRNE ADC_DIVIDE_64				; IF NOT ZERO LOOP BACK FROM LABEL DIVIDE_64
	LDS R16,ADC0_MUXPOS				; COPY MUXPOS REGISTER TO R16
	ANDI R16,0b00000011				; BITWISE AND WITH 0X3
	CPI R16,0b00000011				; COMPARE RESULT WITH 0X3 , IF SAME AIN3 IN REGISTER OTHERWISE ITS AIN2
	BRNE TEMP_STORE
	STS FUEL_BUFFERH,R17			;averaged result in fuel_buffer SRAM (based on muxpos value)
	STS FUEL_BUFFERL,R16			;averaged result in fuel_buffer SRAM
	RET
TEMP_STORE:
	STS TEMP_BUFFERH,R17			;averaged result in temp_buffer SRAM (based on muxpos value)
	STS TEMP_BUFFERH,R16			;averaged result in temp_buffer SRAM
	RET

CHANGE_MUX_POS:
	LDS R16,ADC0_MUXPOS				; COPY MUXPOS REGISTER TO R16
	ANDI R16,0b00000011				; BITWISE AND WITH 0X3
	CPI R16,0b00000011				; COMPARE RESULT WITH 0X3 , IF SAME AIN3 IN REGISTER OTHERWISE ITS AIN2
	BRNE LOAD_AIN3					; IF NOT EQUAL BRANCH TO LABEL LOAD_AIN3
	ORI R16,0b00000010				; IF SAME OR WITH IMMEDIATE 0X2 TO R16 , ONLY BIT 1 & 0 IS CHANGED
	STS ADC0_MUXPOS,R16				; STORE VALUE BACK IN MUXPOS
	RET
LOAD_AIN3:
	ORI R16,0b00000011
	STS ADC0_MUXPOS,R16				; THIS STEP CHANGES THE ADC INPUT FROM AIN2 = PA2 =12 TO AIN3 = PA1 =13
	RET	
					



FUEL_ADC_RANGE_CHECK:				;GM Optra 2009 has full tank at 0.98V = 204ADC count & 2.5V at empty = 520ADC count
	LDI R18,HIGH(102)				;16 bit compare of 204 adc count
	CLC								;clear carry
	CPI R16,102						;16 bit compare
	CPC R17,R18						;16 bit compare with carry
	BRLO SHOW_MAX_READ				;if lower than 204 branch to maximum read (full, 55 liters)
	CLC								; clear carry
	LDI R18,HIGH(520)				;load high of 520 in r18
	CPI R16,LOW(520)				;16 bit compare of result(low) with immediate low(520)
	CPC R17,R18						;compare with carry result(high) and r18 which has high(520)
	BRSH SHOW_LOW_READ				;if same or higher branch to low read (empty)
	RET								;if within range no changes are made to result and return
SHOW_LOW_READ:
	LDI R16,HIGH(520)
	STS FUEL_BUFFERH,R16
	LDI R16,LOW(520)
	STS FUEL_BUFFERL,R16
	RET

SHOW_MAX_READ:
	LDI R16,LOW(102)				;load full tank capacity low(55) liters to r16
	STS FUEL_BUFFERL,R16			;replace previous result from ADC with 55liters
	LDI R16,HIGH(102)				;load full tank capacity high(55) liters to r16
	STS FUEL_BUFFERH,R16			;replace previous result from ADC with 55liters
	RET


/*FUEL_CALC:
lds r16,FUEL_BUFFERL
lds r17,FUEL_BUFFERH
clr r18
clc
subi r16,204
sbc r17,r18
mov r17,r16
ldi r16,0b00010111					;0b00010111 is 0.176 in Q1.7 format fixed point arithmatic. 55/316=.176....
fmul r16,r17						;r17 holds the difference between adc value and full tank value count204
movw r17:r16,r1:r0					;fmul has its result in reg pair r1:r0 which is copied back to r17:r16 .for integer part drop the lower byte which holds fraction 
ldi r16,55							;load immediate value of 55Liters 
sub r16,r17							;subtract consumed fuel value from 55L. 55-((ADC value - 204)*0.5).// 0.5= 55L/(314 -204) = 55L/110 adc steps. each step is approx 0.5L
clr r17								;clear r17 for ASCII conversion
rcall ASCII_CONVERT					; call ASCII_CONVERT subroutine to convert value in r16
ret*/
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;0.5v = 102
;2.5v = 520 (510 approximated to high side)
;2.5-0.5 = 520-102 = 418 steps
;55/418 = 0.131 L/step
;0.131 @ Q7 = 0x11 = 0b00010001
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

FUEL_CALC:
lds r16,FUEL_BUFFERL
lds r17,FUEL_BUFFERH
clr r18
clc
subi r16,102
sbc r17,r18
mov r17,r16
ldi r16,0b00010001					;0b00010001 is 0.131 in Q1.7 format fixed point arithmatic. 55/418 = .131....
fmul r16,r17						;r17 holds the difference between adc value and full tank value count204
movw r17:r16,r1:r0					;fmul has its result in reg pair r1:r0 which is copied back to r17:r16 .for integer part drop the lower byte which holds fraction 
ldi r16,55						;load immediate value of 55Liters 
sub r16,r17						;subtract consumed fuel value from 55L. 
clr r17							;clear r17 for ASCII conversion
rcall ASCII_CONVERT					; call ASCII_CONVERT subroutine to convert value in r16
ret

TEMP_CALC:
LDI R19,LOW(1024)
LDI R20,HIGH(1024)
LDS R16,TEMP_BUFFERL
LDS R17,TEMP_BUFFERH
SUB R19,R16
SBC R20,R17
ldi r16,0xD
fmul r16,r19
mov r19,r1
LDI R16,0xD

fmul r16,r20
mov r20,r1
ldi r17,33
clr r18
clc
add r14,r17
adc r15,r18
mov r16,r14
clr r17
rcall ASCII_CONVERT
ret




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;* Integer to ASCII converter subrountine here
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ASCII_SETUP:
	;LDS R17,ADC_H						; LOAD R17 WITH ADC_H
	;LDS R16,ADC_L						; LOAD R16 WITH ADC_L
ASCII_CONVERT:
	ldi		HundredCounter,0			; set counters values to zero
	ldi		TenthCounter,0				;	
	ldi		OneCounter,0				;
	ldi		ZEROREG,48					; ASCII = Number + 48; In ASCII table, we can see that '0' is 48 (0x30); therefore, need to add 48 to the single digits
	;ldi		intVal,Number
	;rcall	IntToASCII					; subroutine call
										; which convert integer value to
										; ASCII value
IntToASCII:
	LDI R18,HIGH(100)					;16bit comparison
	CLC									;clear carry
	CPI R16,100							;16bit comparison
	CPC R17,R18							;16bit comparison
	;cpi	    intVal,100				; for values upto 0x255 ,8 bit
	brge	DivideBy100					; jump if (intVal>=100) 
	LDI R18,HIGH(10)					;16bit comparison
	CLC									;clear carry
	CPI R16,10							;16bit comparison
	CPC R17,R18							;16bit comparison
	;cpi		intVal,10				;part of 8 bit routine
	brge	DivideBy10					; jump if (intVal>=10)
	mov		OneCounter,r16
	add		HundredCounter,ZEROREG			; here, we got there single digits
	add		TenthCounter,ZEROREG			; from that, we convert them to
	add		OneCounter,ZEROREG				; ASCII values !!!
	ret

DivideBy100:
	;subi	intVal,100
	CLR R19
	CLC
	SUBI R16,100
	SBC R17,R19
	inc		HundredCounter
	rjmp	IntToASCII

DivideBy10:
	;subi	intVal,10
	CLR R19
	CLC
	SUBI R16,10
	SBC R17,R19
	inc		TenthCounter
	rjmp	IntToASCII


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;TIMER
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Note:? To perform a 16-bit write operation, the high byte must be written before the low byte. For a 16-bitread, the low byte must be read before the high byte.
SPEED_INIT:
LDI R16,0b00001101						;enable timer A as we need the clock from this for timer B
STS TCA0_SINGLE_CTRLA,R16				; CLKSEL =0X6 ENABLE 0X1 , PRESCALER 256,39062 COUNTS IN 1 SEC




LDI R16,0b00000101
STS TCB0_CTRLA,R16   					;CKSEL= 0X2 = USE CLK_TCA ENABLE TIMER B
LDI R16,0b00000011
STS TCB0_CTRLB,R16						;ENABLE FREQUENCY MEASURE MODE
LDI R16,0b00010001
STS TCB0_EVCTRL,R16						;ENABLE NEGATIVE EDGE AND START INPUT CAPTURE
LDI R16,0b00000001
STS TCB0_INTCTRL,R16					;ENABLE INPUT CAPTURE INTERRUPT
RET

CAPT_FLAGS:
LDS R16,TCB0_INTFLAGS
SBRS R16,0								;IF BIT SET CAPTURE REGISTER IS LOADED,CLEARED ON READ
RJMP CAPT_FLAGS
RCALL SPEED_READ
RET

CAPT_STATUS:
LDS R16,TCB0_STATUS						;IF 0X01 COUNTER IS RUNNING , 0X00 = OFF
ANDI R16,0b00000001
RET



SPEED_READ:
LDI ZL,LOW(BUFFER)
LDI ZH,HIGH(BUFFER)
LDS R16,TCB0_CCMPL
ST Z+,R16
LDS R16,TCB0_CCMPH
ST Z+,R16
RET

SPEED_STOP:
LDI R16,0X00
STS TCB0_CTRLA,R16
STS TCB0_INTCTRL,R16
STS TCB0_CCMPH,R16
STS TCB0_CCMPL,R16
STS TCB0_CNTH,R16
STS TCB0_CNTL,R16
RET 


SPEED_CALC:
LDI ZL,LOW(BUFFER)
LDI ZH,HIGH(BUFFER)
LD R16,Z+
LD R17,Z 
RCALL division
LDI ZL,LOW(BUFFER)
LDI ZH,HIGH(BUFFER)
LD R16,Z+
LD R17,Z 
rcall ASCII_CONVERT
RET






;***************************************************************************
; DIVISION
;***************************************************************************
;*
;* "div16u" - 16/16 Bit Unsigned Division
;*
;* This subroutine divides the two 16-bit numbers
;* "dd8uH:dd8uL" (dividend) and "dv16uH:dv16uL" (divisor).
;* The result is placed in "dres16uH:dres16uL" and the remainder in
;* "drem16uH:drem16uL".
;*
;* Number of words	:19
;* Number of cycles	:235/251 (Min/Max)
;* Low registers used	:2 (drem16uL,drem16uH)
;* High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;*
;***************************************************************************

;***** Subroutine Register Variables

;r14 = remainder 
;r15 = remainder
;r16 = result   
;r17 = result   
;r16 = dividend  
;r17 = dividend  
;r18 = divisor  
;r19 = divisor  
;r20 = counter  
division:
	push r14
	push r15
	push r16
	push r17
	push r18
	PUSH R19
	PUSH R20
	cpi r16,0				;if result low is 0 exit division to avoid division by 0
	brne start_op			;if result low is 0 exit division to avoid division by 0 = go to label ASCII_CONVERTER
	cpi r29,0				;
	breq no_signal			;even if the high register is valid it will become 0 in strt_op function, ASCII_CONVERTER is also dealing with 8 bits
							;check high register only if 16 bit result is needed to be printed on LCD.our car has max 180kmp which is less than 256.

start_op:
		mov R18,R16
		mov R19,R17
		ldi R16,low(39062)
		ldi R17,high(39062)

;***** Code

div16u:	clr	r14 		;clear remainder Low byte
		sub	r15,r15		;clear remainder High byte and carry
		ldi	r20,17		;init loop counter
d16u_1:	rol	r16			;shift left dividend
		rol	r17
		dec	r20			;decrement counter
		brne	d16u_2			;if done
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;any manipulation of division result ,put code here
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
		lsr 	r17		;    right shift highregister
		ror 	r16		;    rotate carry to low register ,now result is divided by 2 which is same as GPS speed freq/2
no_signal:
		LDI ZL,LOW(BUFFER)
		LDI ZH,HIGH(BUFFER)
		st Z+, r16
		st Z,r17
		pop r20
		pop r19
		pop r18
		pop r17
		pop r16
		pop r15
		pop r14
		ret
						;    return
d16u_2:	rol	r14 		;    shift dividend into remainder
		rol	r15
		sub	r14,r18	;    remainder = remainder - divisor
		sbc	r15,r19		;
		brcc	d16u_3			;    if result negative
		add	r14 ,r18	;    restore remainder
		adc	r15,r19
		clc				;    clear carry to be shifted into result
		rjmp	d16u_1			;    else
d16u_3:	sec				;    set carry to be shifted into result
		rjmp	d16u_1







;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;DELAY SUBROUTINES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
     ; ============================== Time Delay Subroutines =====================
; Name:     delayYx1mS
; Purpose:  provide a delay of (YH:YL) x 1 mS
; Entry:    (YH:YL) = delay data
; Exit:     no parameters
; Notes:    the 16-bit register provides for a delay of up to 65.535 Seconds
;           requires delay1mS

delayYx1mS:
    rcall    delay1mS                        ; delay for 1 mS
    sbiw    YH:YL, 1                        ; update the the delay counter
    brne    delayYx1mS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret
; ---------------------------------------------------------------------------
; Name:     delayTx1mS
; Purpose:  provide a delay of (temp) x 1 mS
; Entry:    (temp) = delay data
; Exit:     no parameters
; Notes:    the 8-bit register provides for a delay of up to 255 mS
;           requires delay1mS

delayTx1mS:
    rcall    delay1mS                        ; delay for 1 mS
    dec     temp                            ; update the delay counter
    brne    delayTx1mS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret

; ---------------------------------------------------------------------------
; Name:     delay1mS
; Purpose:  provide a delay of 1 mS
; Entry:    no parameters
; Exit:     no parameters
; Notes:    chews up fclk/1000 clock cycles (including the 'call')

delay1mS:
    push    YL                              ; [2] preserve registers
    push    YH                              ; [2]
    ldi     YL, low(((fclk/1000)-18)/4)     ; [1] delay counter              (((fclk/1000)-18)/4)
    ldi     YH, high(((fclk/1000)-18)/4)    ; [1]                            (((fclk/1000)-18)/4)

delay1mS_01:
    sbiw    YH:YL, 1                        ; [2] update the the delay counter
    brne    delay1mS_01                     ; [2] delay counter is not zero

; arrive here when delay counter is zero
    pop     YH                              ; [2] restore registers
    pop     YL                              ; [2]
    ret                                     ; [4]

; ---------------------------------------------------------------------------
; Name:     delayTx1uS
; Purpose:  provide a delay of (temp) x 1 uS with a 16 MHz clock frequency
; Entry:    (temp) = delay data
; Exit:     no parameters
; Notes:    the 8-bit register provides for a delay of up to 255 uS
;           requires delay1uS

delayTx1uS:
    rcall    delay1uS                        ; delay for 1 uS
    dec     temp                            ; decrement the delay counter
    brne    delayTx1uS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret

; ---------------------------------------------------------------------------
; Name:     delay1uS
; Purpose:  provide a delay of 1 uS with a 10 MHz clock frequency ;
; Entry:    no parameters
; Exit:     no parameters
; Notes:    add another push/pop for 20 MHz clock frequency

delay1uS:
    push    temp                            ; [1] these instructions do nothing except consume clock cycles
    pop     temp                            ; [2]
    ret                                     ; [4]
     
     

; ============================== End of Time Delay Subroutines ==============            

string:
.db 'H','E','L','L','O',0X00 
string1:
.db 'S','P','E','E','D',0X00
string2:
.db 'T','E','M','P',0X00
string3:
.db 'F','U','E','L',0X00
                                                 