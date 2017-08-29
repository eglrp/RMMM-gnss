        .text

        .code   32
        .align  4

        .include "FRi.asm"
        .include "FR_defines.asm"

        .global gpOS_interrupt_init
        .global interrupt_handler
        .global gpOS_interrupt_lock
        .global gpOS_interrupt_unlock
        .global interrupt_enter
        .global interrupt_leave
        .global reschedule_from_interrupt

    .if CLIBS_RETARGETING == TRUE
        .extern _clibs_memset                         @ from clibs.h
        .extern _clibs_strncpy                        @ from clibs.h
    .else
        .extern _memset                               @ from string.h
        .extern _strncpy                              @ from string.h
    .endif

        .extern gpOS_bsp_interrupt_dispatch_irq       @ from gpOS_bsp.h
        .extern gpOS_bsp_interrupt_clear_irq          @ from gpOS_bsp.h

        .extern gpOS_memory_allocate                  @ from FR_memory.h

        .extern gpOS_bsp_on_interrupt_enter           @ from gpOS_bsp.h
        .extern gpOS_bsp_on_interrupt_leave           @ from gpOS_bsp.h

        .extern vPortYieldProcessor                   @ from portISR.c
        .extern interrupt_occurred_count              @ from port.c

@-----------------------------------------------------------------------
@ void gpOS_interrupt_init( gpOS_partition_t *part, const size_t stack_size)@
@-----------------------------------------------------------------------
gpOS_interrupt_init:
    @
    @ save some scratch registers and lr for return
    @
        stmfd   sp!, {r1-r5,lr}

        mov     r4, r0
        mov     r5, r1

    @
    @ align memory for stack allocation
    @
        mov     r1, #TASK_STACKALIGNMENT
        bl      memory_align_p

    @
    @ align memory size and allocate it
    @
        add     r1, r5, #(TASK_STACKALIGNMENT - 1)
        and     r1, r1, #~(TASK_STACKALIGNMENT - 1)
        mov     r0, r4
        bl      gpOS_memory_allocate_p

    @
    @ check if memory was allocated
    @
        cmp     r0, #0x0
        beq     gpOS_interrupt_init_exit
        mov     r4, r0
    @
    @ set system stack pointer into kernel
    @
        add     r1, r0, r5
        ldr     r2, =modeSYS_sp
        str     r1, [r2]
    @
    @ set system mode stack to some default value
    @
        mov     r2, r5
        mov     r1, #0x55
    .if CLIBS_RETARGETING == TRUE
        bl      _clibs_memset
    .else
        bl      memset
    .endif

    @
    @ write stack name on top of the stack
    @
        mov     r0, r4
        ldr     r1, =interrupt_stack_name
        mov     r2, #TASK_NAME_SIZE
    .if CLIBS_RETARGETING == TRUE
        bl      _clibs_strncpy
    .else
        bl      strncpy
    .endif

    @
    @ init went ok
    @
        mov     r0, #0x1

gpOS_interrupt_init_exit:
        ldmfd   sp!, {r1-r5, lr}          @restore some scratch registers and return
        bx      lr

@-----------------------------------------------------------------------
@ void interrupt_handler( void)@
@-----------------------------------------------------------------------
interrupt_handler:

    @
    @// To replace the __irq directive
    @
        stmfd   sp!,{r0-r12,lr}

    @
    @// IRQ Header
    @
        bl      interrupt_enter

    @
    @// Informs the hw that the ISR has been initiated,
    @// get:
    @// - the irq_line (pointed by r0)
    @// - the handler ptr (pointed by r1)
    @// - the argument ptr (pointed by r2)
    @// on IRQ stack
    @//
        sub     r2, sp, #0x10
        sub     r1, sp, #0xc
        sub     r0, sp, #0x8
        mov     sp, r2
        ldr     r3, =gpOS_bsp_interrupt_dispatch_irq
        mov     lr, pc
        bx      r3

    @//
    @// get back@
    @// - argument ptr in r0
    @// - handler ptr in r1
    @// - irq_line in r2
    @// - dummy value in r3
    @
        ldmfd   sp!, {r0-r3}

    @//
    @// Check if line is safe
    @//
        teq     r2, #0x80000000
        beq     interrupt_handler_fakeirq

    @
    @// ARM IRQ Header
    @
        mrs     r3, spsr                  @// save the spsr into r3
        stmfd   sp!, {r2, r3}             @// save line and msr onto IRQ sp

    @
    @// don't enable IRQs, go into system mode and align the stack
    @
        msr     cpsr_c, #MODE_SYS|I_BIT|F_BIT
        mov     r2, sp
        and     r2, r2, #4
        sub     sp, sp, r2
        stmfd   sp!, {r2, lr}

    .if FREERTOS_PROFILING == TRUE
    @
    @// Increment number of interrupts occurred
    @
        ldr     r2, =interrupt_occurred_count       @ r2 = &interrupt_occurred_count
        ldr     r3, [r2]                            @ r3 = *r2
        add     r3, r3, #1                          @ r3 = r3 + 1
        str     r3, [r2]                            @ *r2 = r3
    .endif

    @
    @// Execute ISR Body routine if any
    @
        msr     cpsr_c, #MODE_SYS    @// reenable IRQs, go into system mode
        cmp     r1, #0
        beq     interrupt_handler_footer

    @
    @// save argument and ISR vector on stack for later usage
    @
        stmfd   sp!, {r0-r1}

    .if FREERTOS_IRQ_HOOKS == TRUE
    @
    @// call interrupt enter hook
    @
        ldr     r2, =gpOS_bsp_on_interrupt_enter
        mov     lr, pc
        bx      r2
    .endif

    @
    @// recall argument and ISR vector from stack, but keep them on it for leave hook usage
    @
        ldmfd   sp, {r0-r1}

    @
    @// call ISR vector
    @

        mov     lr, pc
        bx      r1

    @
    @// pop argument and ISR vector from stack
    @
        ldmfd   sp!, {r0-r1}

    .if FREERTOS_IRQ_HOOKS == TRUE
    @
    @// call interrupt leave hook
    @
        ldr     r2, =gpOS_bsp_on_interrupt_leave
        mov     lr, pc
        bx      r2
    .endif

    @
    @// disable IRQs, go into system mode and align back the stack
    @
        msr     cpsr_c, #MODE_SYS|I_BIT|F_BIT
        ldmfd   sp!, {r2, lr}
        add     sp, sp, r2

interrupt_handler_footer:
    @
    @// ARM IRQ Footer
    @
        msr     cpsr_c, #MODE_IRQ|I_BIT|F_BIT         @// disable IRQs, move to IRQ mode

        ldmfd   sp!, {r2, r3}         @// restore spsr in r3 and line in r0 from IRQ sp (line in r2 to align to fake irq)
        msr     spsr_cxsf, r3         @// restore status register spsr

interrupt_handler_fakeirq:
    @
    @// get IRQ line back from the stack and clear pending bit in hw
    @
        mov     r0, r2
        ldr     r2, =gpOS_bsp_interrupt_clear_irq
        mov     lr, pc
        bx      r2

    @
    @// IRQ Footer
    @
        bl      interrupt_leave

    @
    @// return from IRQ and interrupt reenabled
    @
        ldmfd   sp!,{r0-r12,lr}
        subs    pc,lr,#4


@-----------------------------------------------------------------------
@ void gpOS_interrupt_lock( void);
@-----------------------------------------------------------------------

gpOS_interrupt_lock:
        stmfd   sp!, {r0-r1}          @ save some scratch registers
        mrs     r0, cpsr              @ get cpsr
        ands    r1, r0, #0xf          @ check if USR mode (bit 0-3 should be 0)
        swieq   0x0                   @ if so, call the interrupt disable swi
        orrne   r0, r0, #I_BIT        @ else, disable IRQs (note: this code is
        msrne   cpsr_c, r0            @ not executed when coming back from SWI)

    @
    @ increment lock count
    @
        ldr     r0, =interrupt_lock_count
        ldr     r1, [r0]
        add     r1, r1, #0x1
        str     r1, [r0]

        ldmfd   sp!, {r0-r1}          @restore some scratch registers
        bx      lr

@-----------------------------------------------------------------------
@ void gpOS_interrupt_unlock( void);
@-----------------------------------------------------------------------

gpOS_interrupt_unlock:
        stmfd   sp!, {r0-r1}          @ save some scratch registers

    @
    @ check if the interrupt is locked more than once
    @
        ldr     r1, =interrupt_lock_count
        ldr     r0, [r1]
        subs    r0, r0, #0x1
        strpl   r0, [r1]              @ store counter only if not negative
                                      @ (avoids errors in calling too many unlocks)

        ldmgtfd sp!, {r0-r1}          @ if greater than 0, restore some scratch
        bxgt    lr                    @ registers and return

        mrs     r0, cpsr              @ get cpsr
        ands    r1, r0, #0xf          @ check if USR mode (bit 0-3 should be 0)
        swieq   0x1                   @ if so, call the interrupt enable swi
        bicne   r0, r0, #I_BIT        @ else, enable IRQs (note: this code is
        msrne   cpsr_c, r0            @ not executed when coming back from SWI)

        ldmfd   sp!, {r0-r1}          @ restore some scratch registers
        bx      lr

@-----------------------------------------------------------------------
@ void interrupt_enter(void)
@
@ pre: IRQ mode
@      either running on IRQ stack or on interruptStack if nested interrupt
@      r0-r12,r14 have been saved
@-----------------------------------------------------------------------

interrupt_enter:
    @
    @ save user SP and LR on IRQ stack. Decrement is not done automatically to avoid
    @ warning from assembler about unpredictable value in sp
    @
        stmfd   sp, {r13,r14}^          @ *(irq_sp-8) = user_sp, *(irq_sp-4) = user_lr
        nop
        sub     sp, sp, #8              @ irq_sp -= 8

    @
    @ interrupt_nested_count++
    @
        ldr     r1, =interrupt_nested_count  @ r1 = &interrupt_nested_count
        ldr     r2, [r1]                  @ r2 = *r1
        add     r2, r2, #1                @ r2 = r2 + 1
        str     r2, [r1]                  @ *r1 = r2

    @
    @ if (interrupt_nested_count == 1)
    @   user sp = interruptStack@
    @
        cmp     r2, #1
        bne     interrupt_enter_isnested

        stmfd   sp!, {r0, lr}

    .if FREERTOS_PROFILING == TRUE

    @
    @ stop counting cpu usage for current task
    @

        ldr     r0, =kernel_stats_suspend
        mov     lr, pc
        bx      r0

    .endif

    .if FREERTOS_IRQ_HOOKS == TRUE

    @
    @ execute interrupt enter hook
    @

        ldr     r0, =interrupt_enter_hook
        ldr     r0, [r0]
        cmp     r0, #0x0
        movne   lr, pc
        bxne    r0

    .endif

        ldmfd   sp!, {r0, lr}

        ldr     r1, =modeSYS_sp         @ r1 = &kernel_SYSmode_sp
        ldr     r1, [r1]                @ r1 = kernel_SYSmode_sp
        stmdb   sp, {r1}                @ *(irq_sp-4) = kernel_SYSmode_sp
        ldmdb   sp, {r13}^              @ user_sp = *(irq_sp-4)

interrupt_enter_isnested:
        nop
        bx      lr

@-----------------------------------------------------------------------
@ void interrupt_leave(void)
@
@ pre:
@      running in system mode,
@      interrupt_enter was called
@      sp value is the same as after interrupt_enter called
@-----------------------------------------------------------------------

interrupt_leave:
    @
    @ restore user SP from IRQ stack. Increment is not done automatically to avoid
    @ warning from assembler about unpredictable value in sp
    @
        ldmfd   sp, {r13}^          @ *(irq_sp-8) = user_sp, *(irq_sp-4) = user_lr
        nop
        add     sp, sp, #4              @ irq_sp += 8
        ldmfd   sp, {r14}^          @ *(irq_sp-8) = user_sp, *(irq_sp-4) = user_lr
        nop
        add     sp, sp, #4          @ irq_sp += 8

    @
    @ interrupt_nested_count--
    @
        ldr     r1, =interrupt_nested_count @ r1 = &interrupt_nested_count
        ldr     r2, [r1]                    @ r2 = *r1
        sub     r2, r2, #1                  @ r2 = r2 - 1
        str     r2, [r1]                    @ *r1 = r2

    @
    @ if (interrupt_nested_count != 0)
    @   Continue previous isr, no rescheduling
    @
        cmp     r2, #0
        bne     interrupt_leave_noreschedule

    @
    @ execute interrupt leave hook
    @
        stmfd   sp!, {r0, lr}

    .if FREERTOS_IRQ_HOOKS == TRUE

        ldr     r0, =interrupt_leave_hook
        ldr     r0, [r0]
        cmp     r0, #0x0
        movne   lr, pc
        bxne    r0

    .endif

    .if FREERTOS_PROFILING == TRUE
    @
    @ start counting cpu usage for current task
    @
        ldr     r0, =kernel_stats_restart
        mov     lr, pc
        bx      r0

    .endif

        ldmfd   sp!, {r0, lr}

    @
    @ if (!xReschedule_fromISR_required) goto No_Res - nothing to reschedule
    @
        ldr     r1, =xReschedule_fromISR_required
        ldr     r2, [r1]
        cmp     r2, #1
        bne     interrupt_leave_noreschedule

    @
    @ If I have to reschedule then the APCS restore registers must be
    @ performed here. Then we will jump into kernel_reschedule_from_interrupt
    @ and we will save there all the regs with SAVE_REGISTERS
    @
        mov     r0, #0
        ldr     r1, =xReschedule_fromISR_required
        str     r0, [r1]                @ Reset the xReschedule_fromISR_required
        ldmfd   sp!,{r0-r12,r14}        @ restore APCS registers saved by interrupt handler
        sub     r14,r14,#4              @ subtract 4 to point to interrupt instruction
        b       vPortYieldProcessor
    @
    @should never come back here....
    @

interrupt_leave_noreschedule:
    @
    @ return
    @
        bx      lr

@-----------------------------------------------------------------------
@ Data area
@-----------------------------------------------------------------------

        .data
        .align  4

    @
    @ variable to force OS reschedule from an interrupt handler
    @
        .global xReschedule_fromISR_required
xReschedule_fromISR_required:   .long   0x00000000

    @
    @ interrupt nesting count
    @
        .global interrupt_nested_count
interrupt_nested_count:   .long   0x00000000

    .if FREERTOS_PROFILING == TRUE
    @
    @ Number of interrupts and context swith occurred
    @
        .global interrupt_occurred_count
        .global context_switch_count
interrupt_occurred_count: .long   0x00000000
context_switch_count:     .long   0x00000000
    .endif

    @
    @ interrupt lock count
    @
        .global interrupt_lock_count
interrupt_lock_count:     .long   0x00000000

    @
    @ stack pointer for SYS mode
    @
modeSYS_sp:          .long   0x00000000

    .if FREERTOS_IRQ_HOOKS == TRUE

    @
    @ interrupt enter and leave hooks
    @
        .global interrupt_enter_hook
        .global interrupt_leave_hook
interrupt_enter_hook:     .long   0x00000000
interrupt_leave_hook:     .long   0x00000000

    .endif

    @
    @ name of SYS stack workspace
    @
interrupt_stack_name:     .ascii  "OS_SYS_ws"
        .align  4

        .end
