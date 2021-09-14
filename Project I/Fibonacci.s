@ STM32F4 Discovery - Assembly template
@ Turns on an LED attached to GPIOD Pin 12
@ We need to enable the clock for GPIOD and set up pin 12 as output.

@ Start with enabling thumb 32 mode since Cortex-M4 do not work with arm mode
@ Unified syntax is used to enable good of the both words...

@ Make sure to run arm-none-eabi-objdump.exe -d prj1.elf to check if
@ the assembler used proper instructions. (Like ADDS)

.thumb
.cpu cortex-m4
.syntax unified
@.arch armv7e-m

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Definitions
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Definitions section. Define all the registers and
@ constants here for code readability.

@ Constants
.equ     LEDDELAY,      100000

@ Register Addresses
@ You can find the base addresses for all the peripherals from Memory Map section
@ RM0090 on page 64. Then the offsets can be found on their relevant sections.

@ RCC   base address is 0x40023800
@   AHB1ENR register offset is 0x30

.equ     RCC_BASE,      0x40023800
.equ     RCC_AHB1ENR,   RCC_BASE + 0x30      @ RCC AHB1 peripheral clock register (page 180)
.equ     RCC_APB1ENR,   RCC_BASE + 0x40

@ GPIOD base address is 0x40020C00
@   MODER register offset is 0x00
@   ODR   register offset is 0x14
.equ     GPIOD_MODER,   0x40020C00      @ GPIOD port mode register (page 281)
.equ     GPIOD_ODR,     0x40020C14      @ GPIOD port output data register (page 283)


.equ    first_term,    r2
.equ    second_term,   r3
.equ    i,             r4
.equ    temp,          r5

@ Start of text section
.section .text
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Vectors
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Vector table start
@ Add all other processor specific exceptions/interrupts in order here
	.long    __StackTop                 @ Top of the stack. from linker script
	.long    _start +1                  @ reset location, +1 for thumb mode

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Main code starts from here
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

_start:
	@ Enable GPIOD Peripheral Clock (bit 3 in AHB1ENR register)
	ldr r0, = RCC_AHB1ENR               @ Load peripheral clock register address to r6
	ldr r1, [r0]                        @ Read its content to r1
	orr r1, 0x00000009                  @ Set bit 3 and 0 to enable GPIOD and GPIOA clock
	str r1, [r0]                        @ Store back the result in peripheral clock register

	@ Make GPIOD Pin12 as output pin (bits 25:24 in MODER register)
	ldr r0, = GPIOD_MODER               @ Load GPIOD MODER register address to r6
	ldr r1, [r0]                        @ Read its content to r1
	and r1, 0x00FFFFFF                  @ Clear bits 24, 25 for P12
	orr r1, 0x55000000                  @ Write 01 to bits 24, 25 for P12
	str r1, [r0]                        @ Store back the result in GPIOD MODER register

    @ Set GPIOD Pin12 to 1 (bit 12 in ODR register)
	ldr r0, = GPIOD_ODR                 @ Load GPIOD output data register
	ldr r1, [r0]                        @ Read its content to r1
	and r1, 0x0000                      @ write 1 to pin 12
	str r1, [r0]                        @ Store back the result in GPIOD output data register

    ldr r12,=#0x4FFFE       // 'delay' subroutine takes parameter from r12

main_loop:

    bic r2,   #0xFFFFFFFF   //first_term
    bic r3,   #0xFFFFFFFF   //second_term
    bic r4,   #0xFFFFFFFF   //i
    bic r5,   #0xFFFFFFFF   //temp

    mov r3, #1
    bl group_id
next:
    add r4,#1
    mov r5,r3
    add r3,r2,r3
    mov r2,r5
    bl  display
    cmp r4,#0x13
    bne next
    
b main_loop


/*@@@@@@@@@@@@@@@@@@@@|Subroutines|@@@@@@@@@@@@@@@@@@@@*/


/****************|Time Delay Subroutines|***************/   

delay:
    push {lr}
    mov r0,r12
dec:   
    subs r0, 1
    bne dec
    pop {pc}

delay1:
    push {lr}
    ldr r0, =0x4FFFFE
dec1:   
    subs r0, 1
    bne dec1
    pop {pc}

delay2:
    push {lr}
    ldr r0, =0x77FFFD
dec2:   
    subs r0, 1
    bne dec2
    pop {pc}


/****************|Toggling Subroutines|****************/
toggle:
    push {lr}
	ldr r0, = GPIOD_ODR                 @ Load GPIOD output data register
	ldr r1, [r0]                        @ Read its content to r1
	eor r1, 0xF000                      @ write 1 to pin 12
	str r1, [r0]                        @ Store back the result in GPIOD output data register
    pop {pc}

/*************|Button Reading Subroutine|*************/

button_read:

    push    {lr}
read:
    bl      delay
    ldr     r0,=0x40020010  // GPIOA_IDR
    ldr     r1,[r0]
    and     r1,0x00000001
    cmp     r1,1
    bne     read

read1:
    bl      delay
    ldr     r0,=0x40020010  // GPIOA_IDR
    ldr     r1,[r0]
    and     r1,0x00000001
    cmp     r1,1
    beq     read1
    pop     {pc}

/****************|Loads GPIOD Group ID|****************/

group_id:

push    {lr}
ldr     r0,=#0x0000F000
ldr     r1,=#GPIOD_ODR
str     r0,[r1]
bl      button_read
pop     {pc}

/****************|Encoding BCD Number|****************/

display:

    push    {lr}

    mov     r10,r3      // r10=second_term 

    mov     r0,#1000    // r0=1000
    udiv    r5,r10,r0   // digit3=r10/1000
    mul     r1,r5,r0    // r1=digit3*1000
    subs    r10,r1      // r10=r10-digit3*1000

    cmp     r5,#0
    beq     first

    bl      load_to_GPIO
    bl      delay1
    bl      clear_GPIO
    bl      delay2

first:
    mov     r0,#100     // r0=100
    udiv    r5,r10,r0   // digit2=r10/100
    mul     r1,r5,r0    // r1=digit2*100
    subs    r10,r1      // r10=r10-digit2*100 

    cmp     r5,#0
    beq     second

    bl      load_to_GPIO
    bl      delay1
    bl      clear_GPIO
    bl      delay2

second:
    mov     r0,#10      // r0=10
    udiv    r5,r10,r0   // digit1=r10/10
    mul     r1,r5,r0    // r1=digit1*10
    subs    r10,r1      // r10=r10-digit1*10

    cmp     r5,#0
    beq     third

    bl      load_to_GPIO
    bl      delay1
    bl      clear_GPIO
    bl      delay2

third:
    mov     r5,r10      // digit0=r10
    bl      load_to_GPIO
    bl      delay1
    bl      clear_GPIO

    bl      button_read

    pop     {pc}

/****************|Loading data to GPIOD|***************/
 // Take the data in r5 and shift it to left by 12 then loads it to GPIOD
load_to_GPIO:
    push    {lr}
    ldr     r0,= GPIOD_ODR
    mov     r1,r5          
    lsl     r1,#12
    str     r1,[r0]
    pop     {pc}

/******************|Clearing GPIOD|******************/
clear_GPIO:
    push    {lr}
    ldr     r0,= GPIOD_ODR
    ldr     r1,=#0x00000000
    str     r1,[r0]
    pop     {pc}
