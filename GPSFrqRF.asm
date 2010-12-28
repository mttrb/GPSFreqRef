;		Matt's Version
;
;		GPSFreqRef.asm 	-- Control program for PIC16F628A
;		based GPS-derived 10MHz frequency reference
;		written by Jim Rowe. Last revised 7/02/2007
;		(note that PIC clock is 10MHz, so 1mc = 4 x 100ns = 400ns)

; ***
ERRORLEVEL -302
ERRORLEVEL -306
; ***

	list	P=16F628A
	#include "p16f628A.inc"
	__config h'2123'	; code prot off, RB4 is I/O, RA5 is MCLR, PWRT enabled
						; and clk osc code 011 = external clock in via RA7/CLKIN
	
; storage buffer for data chars received from GPS Rx in NMEA0183 sentences
NMEAstt	equ		h'20'	; first char of sentence goes here
NMEAend	equ		h'6F'	; last char of sentence goes here

; regs for saving context during interrupt servicing
WSave	equ		h'70'	; w reg stored here
SSave	equ		h'71'	; status reg stored here

; variables at data memory locations
Counter1 equ	h'72'	; general purpose counter variable 1
Counter2 equ 	h'73'	; general purpose counter variable 2
Counter3 equ	h'74'	; general purpose counter variable 3
Keycode	 equ	h'75'	; where keyswitch scanning code is stored
Temp1	equ		h'76'	; working storage 1
Temp2	equ		h'77'	; working storage 2
BufPtr	equ		h'78'	; Storage for current GPSRx buffer ptr (in ISR)
TempChr	equ		h'79'	; Temp storage for received GPS char

; Storage reg for flags
Flags	equ		h'7A'	; Bit 0 = temporary display flag (1 = temp display)
						; bits 1-7 = not used

TempDispCtr	equ	h'7B'	; temporary display timing counter
PLLStatus	equ h'7C'	; PLL status indicator (L or U)
PCHiSave	equ h'7D'	; PCLATH saved here during IntServ routine
FSRSave		equ h'7E'	; FSR save here during IntServ routine

; storage registers for current UTC time (Bank1)
UTCh10	equ		h'A0'	; UTC hours - tens
UTCh01	equ		h'A1'	; UTC hours - units
UTCm10	equ		h'A2'	; UTC minutes - tens
UTCm01	equ		h'A3'	; UTC minutes - units
UTCs10	equ		h'A4'	; UTC seconds - tens
UTCs01	equ		h'A5'	; UTC seconds - units

; storage registers for current UTC date (Bank1)
LCLh		equ h'A6'	; LCL hour temp
LCLcount	equ h'A7'	; LCL count
LCLh10		equ h'A8'	; LCL hour - tens
LCLh01		equ h'A9'	; LCL hour - units

; storage registers for Fix latitude (Bank1)
LATd10		equ h'AA'	; Latitude degrees - tens
LATd01		equ h'AB'	; Latitude degrees - units
LATm10		equ h'AC'	; Latitude minutes - tens
LATm01		equ h'AD'	; Latitude minutes - units
LATf10		equ h'AE'	; Latitude minutes - tenths
LATf20		equ h'AF'	; Latitude minutes - hundredths
LATf30		equ h'B0'	; Latitude minutes - thousandths
LATf40		equ h'B1'	; Latitude minutes - 10thousandths
LAThemi		equ h'B2'	; Latitude hemisphere (N or S)

; storage registers for Fix longitude (Bank1)
LNGd100		equ h'B3'	; Longitude degrees - hundreds
LNGd010		equ h'B4'	; Longitude degrees - tens
LNGd001		equ h'B5'	; Longitude degrees - units
LNGm10		equ h'B6'	; Longitude minutes - tens
LNGm01		equ h'B7'	; Longitude minutes - units
LNGf10		equ h'B8'	; Longitude minutes - tenths
LNGf20		equ h'B9'	; Longitude minutes - hundredths
LNGf30		equ h'BA'	; Longitude minutes - thousandths
LNGf40		equ h'BB'	; Longitude minutes - 10thousandths
LNGhemi		equ h'BC'	; Longitude hemisphere (E or W)

; Storage registers for GPS fix data (Bank1)
GPSQual		equ h'BD'	; Quality of GPS fix indicator (0/1/2)
SIUtens		equ h'BE'	; number of satellites in use - tens
SIUunits	equ h'BF'	; number of satellites in use - units
SIVtens		equ h'C0'	; Number of satellites in view - tens
SIVunits	equ h'C1'	; Number of satellites in view - units
AntHt10		equ h'C2'	; Ant height - tens of metres
AntHt01		equ h'C3'	; Ant height - units of metres
AntHtf1		equ h'C4'	; Ant height - tenths of a metre

; -----------------------------------------------------------------
; now program begins

	ORG		h'00'		; normal start vector
	GOTO Initialise
	ORG		h'04'		; interrupt servicing vector
	GOTO IntService
	
Initialise:
	BCF STATUS, RP0		; set for Bank0
	BCF STATUS, RP1
	BCF STATUS, IRP		; also set indir addressing for Banks0&1
	CLRF PORTB			; clear PortB
	CLRF PORTA			; also PortA (turns off LEDs)
	CLRF INTCON			; disable all interrupts also
	MOVLW h'07'			; as well as the comparators
	MOVWF CMCON			; (thank you, John Clarke!)
	CLRF RCSTA			; also clear RCSTA register
	BSF STATUS, RP0		; then set for Bank1, so we can...
	CLRF PIE1			; clear all interrupt mask bits for clean start
	CLRF VRCON			; also make sure VREF is off
	MOVLW h'81'			; set RA0 & RA7 as inputs, other bits as outputs
	MOVWF TRISA			; by loading its config register
	MOVLW h'0A'			; also set RB1 & RB3 as inputs, other bits as outputs
	MOVWF TRISB			; by loading its config register
	BSF OPTION_REG,7	; also make sure PORT B pullups are disabled
	MOVLW h'81'			; now set SPBRG reg for 4800 baud async serial comms
	MOVWF SPBRG
	BSF TXSTA, BRGH		; also set BRGH bit for high bit rate divisor
	BCF TXSTA, SYNC		; clear SYNC bit in TXSTA reg for async mode
	BCF STATUS, RP0		; then back to Bank0 so we can...
	BSF RCSTA, SPEN		; enable USART serial port
	BSF STATUS, RP0		; then back to Bank1
	BCF TXSTA, TXEN		; transmit disabled for the present
	BSF PIE1, RCIE		; enable Rx interrupts by setting RCIE bit
	BCF STATUS, RP0		; finally return to Bank0
	BSF INTCON,PEIE		; enable peripheral interrupts (for USART)	
	BSF RCSTA, CREN		; also enable reception by setting CREN
	CALL CleanSlate		; reset all GPS info for clean display start
	CALL DispInit		; initialise LCD display module
	CALL DispScreen1	; and show main display screen
	BCF Flags,0			; then clear flag (Flags bit 0)
	BSF INTCON,GIE		; and finally enable global interrupts

LoopStart:
	CALL DoAScan		; go scan the function keys
	MOVF Keycode,1		; check if a key was pressed
	BTFSC STATUS,Z 		; skip if Z=0 because Keycode wasn't zero
	GOTO CheckFix		; Z = 1, so no key pressed - go check fix instead
	MOVLW "F"			; key was pressed, so check if it was S1 (F)
	XORWF Keycode,0		; W will now have 0 if it was a match...
	BTFSS STATUS,Z		; skip if Z=1 (i.e., it was an F, and W -> 0)
	GOTO $+3			; Z=0 (because it wasn't an F), so try again
	CALL DispScreen2	; it was an F, so go display screen2
	GOTO CheckFix		; then move on
	MOVLW "A"			; not S1, so check if it was S2 (A)
	XORWF Keycode,0
	BTFSS STATUS,Z		; skip if Z=1 (i.e., it was an A)
	GOTO $+3			; Z=0, so try again
	CALL DispScreen3	; it was an A, so go display screen3 
	GOTO CheckFix		; then move on 
	MOVLW "S"			; not S2, so check if it was S3 (S)
	XORWF Keycode,0
	BTFSS STATUS,Z		; skip if Z=1 (i.e., it was an S)
	GOTO CheckFix		; and move on
	CALL DispScreen4	; it was an S, so go display screen4
CheckFix:
	BSF STATUS, RP0 	; (move to mem Bank 1 in order	
	MOVF GPSQual,0		; to get GPSQual in w,
	BCF STATUS, RP0		; then go back to mem Bank 0)
	XORLW "0"			; now check if it's a 0
	BTFSC STATUS,Z		; skip if Z=0 (because GPSQual <> "0")
	GOTO $+3			; Z=1 so no fix yet, so go turn off LED1
	BSF PORTA,1			; must be 1 (2D fix) or 2 (3D fix), so turn on LED1
	GOTO CheckPLL		; and continue
	BCF PORTA,1			; turn off LED1 to show no fix yet
CheckPLL:
	BTFSC PORTB,3		; skip if RB3 is 0, showing (mostly) PLL is locked
	GOTO NoLock			; RB3 = 1, so PLL not locked. Go process
	BSF PORTA,2			; PLL is locked, so turn on LED2 via RA2
	MOVLW "L"
	MOVWF PLLStatus		; also set PLL status char to L
	GOTO CheckDisplay	; and move on
NoLock:
	BCF PORTA,2			; PLL not locked, so turn off LED2
	MOVLW "U"
	MOVWF PLLStatus		; and set PLL status char to U
CheckDisplay:
	BTFSS Flags,0		; check if we're in a temp display mode
	CALL DispScreen1	; if we're not, refresh normal display
	GOTO LoopStart		; else or then go back to start of loop
;	
;   Program's main loop ends here -- subroutines follow
;   ---------------------------------------------------------
;
BounceWait:
	; routine to pause long enough for switch bounce to end
	MOVLW h'FA'			; set Counter 3 for 250 loops (250ms approx)
	MOVWF Counter3
	CALL Delay1ms		; go away for 1ms
	DECFSZ Counter3, 1	; then decrement Counter3 & skip when zero
	GOTO $-2			; not zero yet, so loop back
	RETURN				; reached zero, so return

CleanSlate:
	; routine to clear GPS info registers A0 - D7 for clean start
	MOVLW "U"			; before we start, put a U into PLLStatus
	MOVWF PLLStatus		; so it starts off correctly also
	MOVLW h'38'			; now set counter for number of regs
	MOVWF Counter3
	MOVLW h'A0'			; also set up address pointer start
	MOVWF FSR
	MOVLW "0"			; now place a 0 in w
	BSF STATUS, RP0		; move to mem Bank1
	MOVWF INDF			; and store 0 in current ptr address
	BCF STATUS, RP0		; then back to Bank0
	DECFSZ Counter3,1	; now decrement counter & skip when zero
	GOTO $+2
	RETURN				; did reach zero, so all done -- return
	INCF FSR,1			; not yet, so increment FSR
	GOTO $-7			; and loop back to continue

ClearLCD:
	;routine to clear LCD and reset address counter
	MOVLW h'01'			; clear display & reset addr ptr
	CALL DispAddress
	CALL Delay160ms		; pause 160ms to give it time to clear
	CALL Delay160ms		; and again, just for tardy LCDs
	RETURN				; then return	
	
Delay1ms:
	;routine to delay approx 1ms (850 x 1.2us = 1020us) before return
	MOVLW h'0A'			; set Counter1 for 10 outer loops
	MOVWF Counter1
OuterLoop:
	MOVLW h'55'			; and Counter2 for 85 inner loops
	MOVWF Counter2
	DECFSZ Counter2, 1	; decrement Counter2 & skip when zero
	GOTO $-1			; not zero yet, so loop back
	DECFSZ Counter1, 1	; did reach zero, so decrement Ctr1
	GOTO OuterLoop		; didn't hit zero, so loop back
	RETURN				; reached zero (10 x 85 loops) so return

Delay10ms:
	;routine to delay approx 10ms before returning
	MOVLW h'0A'			; set Counter3 for 10 outer loops
	MOVWF Counter3
	CALL Delay1ms		; then wait 1ms
	DECFSZ Counter3, 1	; then decrement Counter3 & skip when zero
	GOTO $-2			; not zero yet, so keep going
	RETURN				; reached zero, so return	
	
Delay160ms:
	;routine to delay approx 160ms before returning
	MOVLW h'A0'			; set Counter3 for 160 outer loops
	MOVWF Counter3
	CALL Delay1ms		; then wait 1ms
	DECFSZ Counter3, 1	; then decrement Counter3 & skip when zero
	GOTO $-2			; not zero yet, so keep going
	RETURN				; reached zero, so return	
	
DispAddress:
	;routine to translate & load display address (in w reg) into LCD
	BCF PORTA,3			; first set RS pin of LCD low, for instr/addr
	CALL Nibbles2LCD	; then send addr nibbles to LCD
	BCF PORTA,3			; make sure RS is is left low
	GOTO BusyWait		; then jump to delay 160us before return
	
DisplayData:
	;routine to display a data byte in w reg at the current LCD address
	BSF PORTA,3			; RS pin of LCD high, for sending data
	CALL Nibbles2LCD	; then send data nibbles to LCD

BusyWait:
	; routine to wait until LCD module is not busy, after writing
	MOVLW h'64'			; set delay counter for 100 loops
	MOVWF Counter1		; (should give about 100 x 4 x 0.4 = 160us)
	NOP
	DECFSZ Counter1,1	; decrement counter & skip when zero
	GOTO $-2			; loop back until we reach zero
	RETURN				; then return

DispInit:
	; routine to initialise LCD display module
	CALL Delay160ms		; first wait about 160ms before proceeding
	BCF PORTA,3			; first set LCD's RS line low (RA3) for instr/addr
	BCF PORTA,4			; also set EN line low (RA4)
	MOVLW h'30'			; load init code into RB4-7
	MOVWF PORTB
	CALL ToggleEN		; then toggle EN to write to LCD
	CALL Delay10ms		; then wait about 10ms
	BCF PORTA,3			; make sure RS is still low
	BCF PORTB,4			; now change code to 20h
	CALL ToggleEN		; toggle EN to write
	CALL Delay10ms		; pause 10ms again	
	MOVLW h'28'			; now set LCD functions (4 bit i/f, 2 lines, 5x7 chars)
	CALL DispAddress	; (also delays for 160us)
	MOVLW h'0C'			; also set display mode (disp on, no blink or cursor)
	CALL DispAddress	; (also delays for 160us)
	CALL ClearLCD		; then clear LCD screen (& delay 320ms)
	MOVLW h'06'			; and finally set entry mode (increm addr, no shift)
	CALL DispAddress	; (also delays for 160us)
	RETURN				; should now be set up & ready to go, so leave
	
DispScreen1:
	; routine to display main info on LCD (time, date, Fix & PLL status)
	MOVLW h'80'			; first set address to line 1, char 0
	CALL DispAddress	; (also delays for 160us)
	MOVLW "U"			; then send "UTC "
	CALL DisplayData
	MOVLW "T"
	CALL DisplayData
	MOVLW "C"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	BSF STATUS, RP0
	MOVF UTCh10,0		; now fetch UTC time chars and send them
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF UTCh01,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW ":"			; with colon delimiters 
	CALL DisplayData
	BSF STATUS, RP0
	MOVF UTCm10,0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF UTCm01,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW ":"
	CALL DisplayData
	BSF STATUS, RP0
	MOVF UTCs10,0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF UTCs01,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW " "			; now send " Fx"
	CALL DisplayData
	MOVLW "F"
	CALL DisplayData
	MOVLW "x"
	CALL DisplayData
	BSF STATUS, RP0		; now jump to Bank1 to get
	MOVF GPSQual,0    	; GPS fix quality (0/1/2) in w
	BCF STATUS, RP0		; then back to Bank0
	CALL DisplayData	; and display it
	MOVLW h'C0'			; now move down to line 2
	CALL DispAddress

	MOVLW "L"		
	CALL DisplayData
	MOVLW "C"	
	CALL DisplayData
	MOVLW "L"
	CALL DisplayData

	MOVLW " "	
	CALL DisplayData

	BSF STATUS, RP0
	MOVF LCLh10,0		; now fetch UTC time chars and send them
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LCLh01,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW ":"			; with colon delimiters 
	CALL DisplayData
	BSF STATUS, RP0
	MOVF UTCm10,0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF UTCm01,0
	BCF STATUS, RP0
	CALL DisplayData

	MOVLW " "
	CALL DisplayData
	MOVLW "P"
	CALL DisplayData
	MOVLW "L"
	CALL DisplayData
	MOVLW "L"
	CALL DisplayData
	MOVLW ":"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVF PLLStatus,0	; followed by PLL status char (L or U)
	CALL DisplayData
	RETURN				; before leaving

DispScreen2:
	; routine to display latitude & longitude info for about 20 secs
	; (after S1 has been pressed)
	MOVLW h'28'			; first set temp display mode timing counter
	MOVWF TempDispCtr	; for about 20 seconds (20 x 2 (was 3) GPS sentences)
	BSF Flags,0			; then set temp display mode flag
	MOVLW h'80'			; now set address to line 1, char 0
	CALL DispAddress
	MOVLW "L"			; send "Lat "
	CALL DisplayData
	MOVLW "a"
	CALL DisplayData
	MOVLW "t"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LATd10,0		; followed by curr lat chars
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LATd01,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW h'DE'			; with colon delimiter
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LATm10,0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LATm01,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW "."			; also decimal point
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LATf10,0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LATf20,0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LATf30,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LAThemi,0		; followed by latitude hemi char (N or S)
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW h'C0'			; now move down to line 2
	CALL DispAddress
	MOVLW "L"			; to send "Lng "
	CALL DisplayData
	MOVLW "n"
	CALL DisplayData
	MOVLW "g"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LNGd100,0		; followed by curr longitude chars
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LNGd010,0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LNGd001,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW 	h'DE'		; again with colon delimiter
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LNGm10, 0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LNGm01, 0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW "."			; also decimal point
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LNGf10,0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LNGf20,0
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LNGf30,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	BSF STATUS, RP0
	MOVF LNGhemi,0		; and finally longitude hemi char (E or W)
	BCF STATUS, RP0
	CALL DisplayData
	RETURN				; before leaving
	
DispScreen3:
	; routine to display ant height, no of satellites in view
	; for about 20 seconds after S2 has been pressed
	MOVLW h'28'			; first set temp display mode timing counter
	MOVWF TempDispCtr	; for about 20 seconds (20 x 2 was 3) GPS sentences)
	BSF Flags,0			; then set temp display mode flag
	MOVLW h'80'			; now set address to line 1, char 0
	CALL DispAddress
	MOVLW "A"			; send "Ant "
	CALL DisplayData
	MOVLW "n"
	CALL DisplayData
	MOVLW "t"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	BSF STATUS, RP0
	MOVF AntHt10,0		; and 10s of metres
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF AntHt01,0		; and units of metres
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW "."			; followed by decimal point
	CALL DisplayData
	BSF STATUS, RP0
	MOVF AntHtf1,0		; and tenths of metre
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW "m"			; followed by "m abvMSL"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW "a"
	CALL DisplayData
	MOVLW "b"
	CALL DisplayData
	MOVLW "v"
	CALL DisplayData
	MOVLW "M"
	CALL DisplayData
	MOVLW "S"
	CALL DisplayData
	MOVLW "L"
	CALL DisplayData
	MOVLW h'C0'			; now move to start of second line
	CALL DispAddress


	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData

	RETURN				; before leaving

DispScreen4:
	; routine to display satellite info in brief form for 20 secs
	; (after S3 is pressed)
	MOVLW h'28'			; first set temp display mode timing counter
	MOVWF TempDispCtr	; for about 20 seconds (20 x 2 (was 3) GPS sentences)
	BSF Flags,0			; then set temp display mode flag
	MOVLW h'80'			; then set address to line 1, char 0
	CALL DispAddress


	MOVLW "S"			; and send "Sats in View: "
	CALL DisplayData
	MOVLW "a"
	CALL DisplayData
	MOVLW "t"
	CALL DisplayData
	MOVLW "s"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW "i"
	CALL DisplayData
	MOVLW "n"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW "U"
	CALL DisplayData
	MOVLW "s"
	CALL DisplayData
	MOVLW "e"
	CALL DisplayData
	MOVLW ":"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	BSF STATUS, RP0
	MOVF SIUtens,0		; now send SIVtens and SIVunits
	BCF STATUS, RP0
	CALL DisplayData
	BSF STATUS, RP0
	MOVF SIUunits,0
	BCF STATUS, RP0
	CALL DisplayData
	MOVLW " "
	CALL DisplayData


	MOVLW h'C0'			; now move to start of second line
	CALL DispAddress

	MOVLW "V"
	CALL DisplayData
	MOVLW "K"
	CALL DisplayData
	MOVLW "6"
	CALL DisplayData
	MOVLW "M"
	CALL DisplayData
	MOVLW "R"
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData
	MOVLW " "
	CALL DisplayData

	RETURN				; and return

DoAScan:
	; routine to scan keys, verify value for valid keypress
	; returns with code (or zero if no key pressed) saved in Keycode
	CALL ScanKeys		; go scan keyboard
	MOVWF Keycode		; then save return value in w to Keycode
	MOVF Keycode,1		; now check if Keycode = 0
	BTFSC STATUS, Z		; skip if Z=0 (ie Keycode not 0)
	RETURN				; otherwise return, as no key was pressed
	CALL BounceWait		; key was pressed, so wait for 250ms
	CALL ScanKeys		; & scan again to verify
	ADDLW h'00'			; test return value in w reg
	BTFSC STATUS,Z		; skip if Z=0 (ie w not 0, has a keycode)
	RETURN				; was zero, so false alarm - exit with w=0
	XORWF Keycode, 0	; not zero, so compare with first code
	BTFSC STATUS,Z		; skip if Z=0 (because codes didn't match)
	RETURN				; did match, so return with code in Keycode
	CALL BounceWait		; no match, so wait another 250ms
	GOTO DoAScan		; and back to scan again

Nibbles2LCD:
	; routine to separate nibbles of data byte in w & send to LCD
	MOVWF Temp2			; first save byte in Temp2
	ANDLW h'F0'			; also strip off lower nibble, then
	MOVWF PORTB			; send hi nibble to LCD via RB4-7
	CALL ToggleEN		; then toggle EN to write into LCD
	SWAPF Temp2,0		; now get byte back in w with nibbles swapped
	ANDLW h'F0'			; strip off lower nibble again, then
	MOVWF PORTB			; and send hi (lower) nibble to LCD
	CALL ToggleEN		; toggle EN line again to write into LCD
	RETURN				; then return
	
ScanKeys:
	; routine to look for a pressed key, return with code in W
	; (or with 0 in w if no key is pressed)
	BSF PORTB, 7		; enable RB7 output to check for S2
	NOP					; short delay (800ns)
	NOP
	BTFSS PORTA, 0		; check if S2 (view Ant Ht) key pressed
	GOTO $+3			; no, so keep looking
	BCF PORTB, 7		; yes, so clear RB7 output
	RETLW "A"			; and return with A in w reg
	BCF PORTB, 7		; nothing yet, so clear RB7 again
	BSF PORTB, 6		; and set RB6 instead
	NOP
	NOP
	BTFSS PORTA, 0		; check if S3 (view SatInfo) key pressed
	GOTO $+3			; no, so keep looking
	BCF PORTB, 6		; yes, so clear RB6 output
	RETLW "S"			; and return with S in w reg
	BCF PORTB, 6		; nothing yet, so clear RB6 again
	BSF PORTB, 5		; and set RB5 instead
	NOP
	NOP
	BTFSS PORTA, 0		; check if S4 (Init GPS Rx) key is pressed
	GOTO $+3			; no, so keep looking
	BCF PORTB, 5		; yes, so clear RB5 output
	RETLW "I"			; and return with I in W reg
	BCF PORTB, 5		; nothing yet, so clear RB5 again
	BSF PORTB, 4		; and set RB4 instead
	NOP
	NOP
	BTFSS PORTA, 0		; check if S1 (view Fix) key is pressed
	GOTO $+3			; no, so depart
	BCF PORTB, 4		; yes, so clear RB4 output
	RETLW "F"			; and return with F in W reg
	BCF PORTB, 4		; no key pressed, so clear RB4 again
	RETLW h'00'			; and return with null in W reg

SendChar:
	; routine to send char in W reg to USART and then out via serial
	; output pin RB2 (to the GPS module, in this case)
	MOVWF TXREG			; first load char in w -> TXREG, to start sending
	BSF STATUS, RP0		; now set for Bank1
	BTFSS TXSTA,1		; skip if TRMT bit in TXSTA register is set
	GOTO $-1			; but if still zero, loop back (still sending)
	BCF STATUS, RP0		; bit is set (char gone), so return to Bank0
	RETURN				; then return 
		
ToggleEN:
	;routine to toggle EN line of LCD, to write an instr or data nibble
	BSF PORTA,4			; take LCD's EN line high (RA4)
	NOP					; pause 0.8us (2mc) to let it stabilise
	NOP
	BCF PORTA,4			; then low again, to write into LCD controller
	RETURN				; then return
		
; ----------------------------------------------------------------	
	
	; end of main routines -- interrupt servicing routines follow
	
IntService:
	; routine to service interrupts from USART
	; when a character has been received from the GPS module
	MOVWF WSave			; first save context (w and status regs)
	SWAPF STATUS,0		; using SWAPF here to avoid STATUS change
	MOVWF SSave
	MOVF PCLATH,0		; also save PCLATH (PC hi bits) so we can
	MOVWF PCHiSave		; restore them before we leave
	MOVF FSR,0			; also save FSR reg for restoring on exit
	MOVWF FSRSave
	BCF STATUS, RP0		; and reset STATUS bits for data memory Bank0
	BCF STATUS, RP1
	BCF STATUS, IRP
	MOVF RCSTA,0		; now start by reading RCSTA register into w
	ANDLW h'06'			; mask off all except error flag bits (1,2)
	BTFSS STATUS,Z		; if Z=1, skip because no error
	GOTO Back4More		; but if Z=0, an error must have occurred so leave
	BTFSS PIR1,RCIF		; no error, so see if RCIF is set
	GOTO Back4More		; if it wasn't just depart again
	MOVF RCREG,0		; it was, so read char from RCREG (clears RCIF)
	MOVWF TempChr		; and save in TempChr
	BSF RCSTA,CREN		; set CREN again to re-enable USART receiving
	MOVLW h'2A'			; now check if char was an asterisk
	XORWF TempChr,0
	BTFSC STATUS,Z		; skip if it wasn't
	GOTO ParseNSave		; Z=1, so must be an * - go parse & save sentence	
	MOVLW h'0D'			; Z=0, so wasn't an *. Now check if it's a CR
	XORWF TempChr,0
	BTFSC STATUS,Z		; skip if Z=0, because it wasn't a CR	
	GOTO ResetBufPtr	; it was a CR, so just reset buf ptr & leave
	MOVLW h'0A'			; wasn't a CR, so check if it's a LF instead
	XORWF TempChr,0
	BTFSC STATUS,Z		; skip if Z=0, because it wasn't a LF
	GOTO ResetBufPtr	; must have been an LF, so reset ptr & leave
	MOVLW h'24'			; not LF either, so see if it's a "$" (line start)
	XORWF TempChr,0
	BTFSC STATUS,Z		; skip if Z=0, because it wasn't a $
	GOTO ResetBufPtr	; must have been a $, so reset buf ptr & leave
	MOVF BufPtr,0		; wasn't any of the above, so fetch buf pointer in w
	MOVWF FSR			; and load into FSR (indir addr reg)
 	MOVF TempChr,0		; then fetch char itself back into w
	MOVWF INDF			; and save it in current buffer location (via FSR)
	INCF FSR, 0			; then incr buf pointer & load into w
	GOTO SavePtrNGo		; ready to save in BufPtr and return

ParseNSave:
	; routines to read NMEA sentence buffer and update saved GPS info
	MOVLW h'20'			; first load FSR with buffer start addr
	MOVWF FSR
	MOVF INDF,0			; now read buffer's first char into w reg
	XORLW "G"			; and check if it's a G
	BTFSS STATUS,Z		; if Z=1, must be a G so skip
	GOTO Depart			; not a G, so skip sentence altogether
	INCF FSR,1			; if it was a G, now check third char
	INCF FSR,1
	MOVF INDF,0
	XORLW "R"			; and check if it's an R
	BTFSS STATUS,Z		; if Z=1, must be an R so skip
	GOTO CheckaG		; Z=0, so it was a G - go parse further

	GOTO Depart

CheckaG:
	MOVLW h'23'			; now load FSR with 4th char address in buffer
	MOVWF FSR
	MOVF INDF,0			; and read that char into W
	XORLW "G"			; and check if it's a G
	BTFSS STATUS,Z		; if Z=1, must be a G so skip
	GOTO Depart		; Z=0, so it can't be GGA so depart 

	MOVLW h'47'			; Z=1, so must be a GPGGA sentence. Continue
	MOVWF FSR
	MOVF INDF,0			; fetch the GPS fix quality index (0/1/2)
	BSF STATUS, RP0		; and save it in GPSQual
	MOVWF GPSQual
	BCF STATUS, RP0
	XORLW "0"			; now just check if it's a zero
	BTFSC STATUS,Z		; skip if Z=0 (because it wasn't)
	GOTO Depart			; but if Z=1, scrap remainder of sentence	



	MOVLW h'26'			; it was an R, so set FSR to read first GPS time char
	MOVWF FSR			; (because it'a a GPRMC sentence)
	MOVF INDF,0			; now read it
	BSF STATUS, RP0		; set for Bank1
	MOVWF UTCh10		; and save it in UTCh10 (tens of hours)
	BCF STATUS, RP0		; then back to Bank0
	INCF FSR,1			; then fetch and save other UTC time chars
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF UTCh01		; the units of hours
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF UTCm10		; the tens of minutes
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF UTCm01		; the units of minutes
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF UTCs10		; the tens of seconds
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF UTCs01		; the units of seconds


	MOVLW h'30'			; if fix is OK, fetch and save latitude digits
	MOVWF FSR
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LATd10		; the tens of degrees
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LATd01		; the units of degrees
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LATm10		; the tens of minutes
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LATm01		; the units of minutes
	BCF STATUS, RP0
	MOVLW h'35'
	MOVWF FSR
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LATf10		; the tenths of minutes
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LATf20		; the hundredths of minutes
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LATf30		; the thousandths of minutes
	BCF STATUS, RP0
	INCF FSR,1
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LAThemi		; the latitude hemisphere (S or N)
	BCF STATUS, RP0
	MOVLW h'3B'			; then fetch and save longitude digits
	MOVWF FSR
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LNGd100		; the hundreds of degrees
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LNGd010		; the tens of degrees
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LNGd001		; the units of degrees
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LNGm10		; the tens of minutes
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LNGm01		; the units of minutes
	BCF STATUS, RP0
	MOVLW h'41'
	MOVWF FSR
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LNGf10		; the tenths of minutes
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LNGf20		; the hundredths of minutes
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LNGf30		; the thousandths of minutes
	BCF STATUS, RP0
	MOVLW h'45'
	MOVWF FSR
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF LNGhemi		; the longitude hemisphere (E or W)
	BCF STATUS, RP0


	MOVLW h'49'			; not a zero, so set FSR for no of sats in use
	MOVWF FSR
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF SIUtens		; no of satellites in use - tens
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF SIUunits		; no of satellites in use - units
	BCF STATUS, RP0
	MOVLW h'51'
	MOVWF FSR
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF AntHt10		; antenna height above MSL - tens
	BCF STATUS, RP0
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF AntHt01		; antenna height above MSL - units
	BCF STATUS, RP0
	INCF FSR,1
	INCF FSR,1
	MOVF INDF,0
	BSF STATUS, RP0
	MOVWF AntHtf1		; antenna height above MSL - 10ths of a metre
	BCF STATUS, RP0

	GOTO	Add8

	GOTO Depart			; that's it for a GPGGA sentence, so leave

Add8:
	BSF STATUS, RP0

	MOVF	UTCh10,0	; Copy UTCh10 to LCLh10
	MOVWF	LCLh10
	MOVLW	"0"
	SUBWF	LCLh10, 1	; convert ASCII number to actual value

	MOVF	UTCh01,0	; do the same for h01
	MOVWF	LCLh01
	MOVLW	"0"
	SUBWF	LCLh01, 1

	MOVLW	0
	ADDWF	UTCh10,0
	ADDWF	UTCh10,0
	ADDWF	UTCh10,0
	ADDWF	UTCh10,0
	ADDWF	UTCh10,0
	ADDWF	UTCh10,0
	ADDWF	UTCh10,0
	ADDWF	UTCh10,0
	ADDWF	UTCh10,0
	ADDWF	UTCh10,0
	ADDWF	UTCHh01,0
	ADDLW	8

	MOVWF	LCLh10
	
	MOVLW	"A"
	ADDWF	LCLh10,1
	BCF STATUS, RP0
	

Depart:
	; exit after parsing a recv'd NMEA sentence & saving or discarding data
	BTFSS Flags,0			; first check if we're in temporary display mode
	GOTO ResetBufPtr		; if we're not, just reset buf pointer & leave
	DECFSZ TempDispCtr,1	; we are, so decrement the temp display timing counter
	GOTO ResetBufPtr		; and depart if it didn't hit zero
	BCF Flags,0				; did hit zero, so clear temp disp mode flag - time's up
ResetBufPtr:
	MOVLW h'20'				; reset sentence buffer pointer for a new sentence
SavePtrNGo:
	MOVWF BufPtr			; save w in BufPtr for saving next char in buffer
	GOTO Back2Reality		; and leave to wait for another char
Back4More:
	; exit after finding USART comms error
	MOVF RCREG,0			; read char from RCREG so RCIF is cleared
	BCF RCSTA,CREN			; clear CREN bit in RCSTA (after receive error)
	BSF RCSTA,CREN			; then set it again to re-enable reception
Back2Reality:
	MOVF FSRSave, 0			; fetch saved FSR contents -> w
	MOVWF FSR				; and restore them to FSR
	MOVF PCHiSave,0			; fetch saved hi bits from PCLATH -> w
	MOVWF PCLATH			; and restore them to PCLATH
	SWAPF SSave,0			; now restore context (status & w regs)
	MOVWF STATUS			;
	SWAPF WSave,1			; using SWAPFs to avoid changing STATUS
	SWAPF WSave,0			;
	RETFIE					; and return, re-enabling ints (TOS->PC, 1->GIE)
	
	END
	
