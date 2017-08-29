@
@ crt0.asm
@
@ Implements hooks for GNU newlib library for
@ - code/data relocation
@ - stack/heap initialization
@
@

        .include "os20_defines.asm"
        .include "mapping_sta8090.asm"

        .text
        .code   32
        .align  4

        .section .reset_vector_area, "ax"

        .extern     gpOS_memory_init

        .extern     LLD_ARM946_Reset

        .extern     platform_usb_sense_init
        .extern     platform_setup
        .extern     gpOS_bsp_reset_handler
        .extern     main

        .extern     os20_root_task_stack_size
        .extern     __os_stack_area_limit__

        .global     reset_vector
        .global     reset_handler
        .global     _sys_exit
        .global     _exit

reset_vector:
        ldr     pc, reset_handler_ptr

reset_handler_ptr:      .long       reset_handler

        .section .reset_handler, "ax"

reset_handler:
    @
    @ --- Reset ARM status to safe condition
    @
        msr     cpsr_c, #gpOS_MODE_SVC|gpOS_I_BIT|gpOS_F_BIT         @// disable IRQs, move to SVC mode

    @
    @ --- Reset USB status to prevent host enumeration
    @
        ldr     r0,=platform_usb_sense_init
        blx     r0

    @
    @ Reset micro
    @
        ldr     r0,=LLD_ARM946_Reset
        blx     r0

    @
    @ --- Configure data TCM to have some stack
    @
        ldr     r0, =(DTCM_START_ADDR|0x12)
        mcr     p15,0,r0,c9,c1,0

        mrc     p15,0,r0,c1,c0,0
        mov     r2,#0x00010000
        orr     r0,r0,r2
        mcr     p15,0,r0,c1,c0,0

    @
    @ --- Provide stack for memory setup step
    @
        ldr     r0, =(DTCM_START_ADDR+DTCM_SIZE_MIN)
        add     r0, r0, #0x4
        and     r0, r0, #0xFFFFFFF8
        mov     sp, r0

    @
    @ --- Execute specific platform setup
    @
        ldr     r0, =platform_setup
        blx     r0

    @
    @ --- Execute specific bsp reset
    @
        ldr     r0, =gpOS_bsp_reset_handler
        blx     r0

    @
    @ Code relocation.
    @ NOTE: It assumes that the DATA size is a multiple of 4.
    @
        ldr     r1, =__code_load_reg_start__
        ldr     r2, =__code_exec_reg_start__
        ldr     r3, =__code_exec_reg_end__
__code_reloc_loop:
        cmp     r2, r3
        ldrlo   r0, [r1], #4
        strlo   r0, [r2], #4
        blo     __code_reloc_loop

    @
    @ Data relocation.
    @ NOTE: It assumes that the DATA size is a multiple of 4.
    @
        ldr     r1, =__data_load_reg_start__
        ldr     r2, =__data_exec_reg_start__
        ldr     r3, =__data_exec_reg_end__
__data_reloc_loop:
        cmp     r2, r3
        ldrlo   r0, [r1], #4
        strlo   r0, [r2], #4
        blo     __data_reloc_loop

    @
    @ BSS initialization.
    @ NOTE: It assumes that the BSS size is a multiple of 4.
    @
        mov     r0, #0
        ldr     r1, =__bss_start__
        ldr     r2, =__bss_end__
__bssloop:
        cmp     r1, r2
        strlo   r0, [r1], #4
        blo     __bssloop

    @
    @ Data (from SRAM2) relocation.
    @ NOTE: It assumes that the DATA size is a multiple of 4.
    @
        ldr     r1, =__data_sram2_load_reg_start__
        ldr     r2, =__sram2_start__
        ldr     r3, =__sram2_end__
__data_reloc_sram2_loop:
        cmp     r2, r3
        ldrlo   r0, [r1], #4
        strlo   r0, [r2], #4
        blo     __data_reloc_sram2_loop

    @
    @ Data (from SRAM) relocation.
    @ NOTE: It assumes that the DATA size is a multiple of 4.
    @
        ldr     r1, =__data_sram_load_reg_start__
        ldr     r2, =__sram_start__
        ldr     r3, =__sram_end__
__data_reloc_sram_loop:
        cmp     r2, r3
        ldrlo   r0, [r1], #4
        strlo   r0, [r2], #4
        blo     __data_reloc_sram_loop

    @
    @ Reserve stack size for root task from OS20 heap region
    @
        ldr     r0, =__os_stack_area_limit__
        add     r0, r0, #0x4
        and     r0, r0, #0xFFFFFFF8

        mov     sp, r0
        ldr     r0, =os20_root_task_stack_size
        ldr     r0, [r0]
        sub     sl, sp, r0

    @
    @ Clear root task stack
    @

        mov     r0, sl
        ldr     r2,=0xAAAAAAAA

__stackheapinit_stack_cycle:
        str     r2, [r0]
        add     r0, r0, #0x4
        cmp     r0, sp
        bne     __stackheapinit_stack_cycle

    @
    @ Reserve space on root stack for registers saving
    @

        sub     sp, sp, #(gpOS_KRNL_REGS_NUMOFREGS << 2)

    @
    @ Initialize memory module for heap configuration (needed for static C++ objects)
    @
        stmfd   sp!, {r0, lr}

        ldr     r0, =gpOS_memory_init
        mov     lr, pc
        bx      r0
        ldmfd   sp!, {r0, lr}

    @
    @ Main program invocation.
    @

        ldr     r12, =main
        mov     lr, pc
        bx      r12

_sys_exit:
_exit:
        b       _exit

        .end
