        .text

        .code   32
        .align  4

        .include "FRi.asm"
        .include "FR_defines.asm"

        .global vectors_init_stacks

@-----------------------------------------------------------------------
@ vectors_init_stacks( vectors_stack_table, vectors_stacks)
@
@ Pre: must be in supervisor mode
@ Post: exits in user mode,
@-----------------------------------------------------------------------

vectors_init_stacks:
    @
    @ save current SP and LR in r2, r3. SP pointed area will be used as stack
    @ for root task.
    @
        stmfd   sp!, {r0-r6}
        mrs     r6, cpsr                                              @ save current cpsr
        mov     r2, sp
        mov     r3, lr
    @
    @ set the SVC mode stack and save in modeSVC_sp
    @
        ldmia   r0!, {r4}                                             @ r4 = SVC stack bottom
        ldmia   r1!, {r5}                                             @ r5 = size of SVC stack top
        add     r4, r4, r5                                            @ r4 = r5 + r4 (SVC stack top)
        mov     sp, r4                                                @ sp = SVC stack top
        ldr     r4, =modeSVC_sp                                       @ r4 = &modeSVC_sp
        str     sp, [r4]                                              @ *r4 = sp@
    @
    @ set the UNDEF mode stack
    @
        msr     cpsr_c, #MODE_UND|I_BIT|F_BIT                         @ change to UNDEF mode with interrupts disabled
        ldmia   r0!, {r4}                                             @ r4 = UNDEF stack bottom
        ldmia   r1!, {r5}                                             @ r5 = size of UNDEF stack top
        add     r4, r4, r5                                            @ r4 = r5 + r4 (UNDEF stack top)
        mov     sp, r4                                                @ sp = UNDEF stack top
    @
    @ set the ABORT mode stack
    @
        msr     cpsr_c, #MODE_ABT|I_BIT|F_BIT                         @ change to ABT mode with interrupts disabled
        ldmia   r0!, {r4}                                             @ r4 = ABT stack bottom
        ldmia   r1!, {r5}                                             @ r5 = size of ABT stack top
        add     r4, r4, r5                                            @ r4 = r5 + r4 (ABT stack top)
        mov     sp, r4                                                @ sp = ABT stack top
    @
    @ set the IRQ mode stack and save in modeIRQ_sp
    @
        msr     cpsr_c, #MODE_IRQ|I_BIT|F_BIT                         @ change to IRQ mode with interrupts disabled
        ldmia   r0!, {r4}                                             @ r4 = IRQ stack bottom
        ldmia   r1!, {r5}                                             @ r5 = size of IRQ stack top
        add     r4, r4, r5                                            @ r4 = r5 + r4 (IRQ stack top)
        mov     sp, r4                                                @ sp = IRQ stack top
        ldr     r4, =modeIRQ_sp                                       @ r4 = &modeIRQ_sp
        str     sp, [r4]                                              @ *r1 = sp
    @
    @ set the FIQ mode stack
    @
        msr     cpsr_c, #MODE_FIQ|I_BIT|F_BIT                         @ change to FIQ mode with interrupts disabled
        ldmia   r0!, {r4}                                             @ r4 = FIQ stack bottom
        ldmia   r1!, {r5}                                             @ r5 = size of FIQ stack top
        add     r4, r4, r5                                            @ r4 = r5 + r4 (FIQ stack top)
        mov     sp, r4                                                @ sp = FIQ stack top
    @
    @ go into user mode with interrupts disabled and restore the sp and lr
    @
        msr     cpsr_c, r6                                            @ restore current cpsr
        mov     sp, r2
        mov     lr, r3
        ldmfd   sp!, {r0-r6}
        bx      lr

        .end
