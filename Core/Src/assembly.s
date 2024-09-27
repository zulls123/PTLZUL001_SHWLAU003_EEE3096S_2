/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT    @load value to set gpiob port as outputs into R2
	STR R2, [R1, #0]        @store into moder register
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

    @Make R7 to store the state of the current LED to use in future, initialized to 0
    MOVS R7, #0

@ TODO: Add code, labels and logic for button checks and LED patterns


main_loop:

    LDR R1, [R0, #0b00010000] @Load IDR register value to check the button states
    MOVS R3, #1 @Bitmask for button 0 in R3 - makes everything else 0, but button 0
    ANDS R1,R1,R3 @Bitwise AND to R1 and R3, storing result in R1 (0b00000001 & 0b00000001)
    CMP R1, #0 @check if SW0 pressed by comparing the value in R1 with 0, if R1 is 0, button not pressed
    BNE increment_leds_by_1 @if SW0 was pressed increment by 1
    

increment_leds_by_1:
    ADDS R2, R2, #1 @increments the led value in R2 by 1
    MOVS R7, R2 @take the updated R2 LED value and put it into R4
    B write_leds

@for when SW0 is pressed
increment_leds_by_2:
    ADDS R2, R2, #2 @increment led value by 2
    MOVS R7, R2 @take the R2 and put it in R7 to be used at another point in time
    B write_leds @branch to write leds


write_leds:
	LDR R5, GPIOB_BASE @take GPIOB base address
    STR R2, [R5, #0x14] @offset by the value for the output register
	

@Button Press Checking
check_button_1:
    LDR R6, [R0, #0x10] @reading input data register
    MOVS R3, #0b00000010 @put immediate value 2 into R3 for checking button 1 bit 1 state
    ANDS R6, R6, R3 @bitwise AND between R6 (IDR) and R3
    CMP R6, #0 @checks SW1 and if it is pressed (if it is = 0)
    BEQ shorter_delay_method @when SW1 pressed, implement short delay

check_button_2:
    LDR R6, [R0, #0x10]
    MOVS R3, #0b00000100  @check state of SW2, mask for bit 1 #0b00000100
    ANDS R6, R6, R3 @bitwise and
    CMP R6, #0
    BEQ set_0xAA @if SW2 is pressed then freeze on this pattern

check_button_3: @SW3
    LDR R6, [R0, #0x10] @Read IDR 
    MOVS R3, #0b00001000
    ANDS R6, R6, R3
    CMP R6, #0 @check if button is pressed
    BEQ button_3_freeze_pattern @freeze current pattern if the compare was equal


@delay labels

longer_delay_method:
    MOVS R2, R7
    LDR R4, LONG_DELAY_CNT
    B decrement_delay_count


shorter_delay_method:
    LDR R4, SHORT_DELAY_CNT @load delay count into R4



decrement_delay_count:
@decrement delay count
    SUBS R4, R4, #1 @R4 = R4 -1, decreases delay by 1
    BEQ main_loop @branch back to main loop
    B decrement_delay_count

@setting patterns labels
set_0xAA:
    MOVS R2, #0xAA @change the value of the LEDs
    B write_leds @ call write leds method to implement led pattern

button_3_freeze_pattern:
    B write_leds @write the same LED value
    B check_button_3 @keep checking if the button is pressec




@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000
GPIOB_BASE:  		.word 0x48000400
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays
LONG_DELAY_CNT: 	.word 1400000
SHORT_DELAY_CNT: 	.word 600000

