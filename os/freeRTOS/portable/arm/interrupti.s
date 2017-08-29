       AREA gpOS_interrupt, CODE, READONLY

        GET FRi.s
        GET FR_defines.s

        EXPORT gpOS_interrupt_init
        EXPORT interrupt_handler
        EXPORT gpOS_interrupt_lock
        EXPORT gpOS_interrupt_unlock
        EXPORT interrupt_enter
        EXPORT interrupt_leave
        EXPORT interrupt_occurred_count

    IF :DEF: CLIBS_RETARGETING
        IMPORT _clibs_memset                         ; from clibs.h
        IMPORT _clibs_strncpy                        ; from clibs.h
    ELSE
        IMPORT memset                               ; from string.h
        IMPORT strncpy                              ; from string.h
    ENDIF

        IMPORT gpOS_bsp_interrupt_dispatch_irq       ; from gpOS_bsp.h
        IMPORT gpOS_bsp_interrupt_clear_irq          ; from gpOS_bsp.h

        IMPORT gpOS_memory_allocate                  ; from FR_memory.h
        IMPORT gpOS_memory_allocate_p                ; from FR_memory.c
        IMPORT memory_align_p                        ; from FR_memoryi.h

        IMPORT gpOS_bsp_on_interrupt_enter           ; from gpOS_bsp.h
        IMPORT gpOS_bsp_on_interrupt_leave           ; from gpOS_bsp.h

        IMPORT vPortYieldProcessor                   ; from portISR.c
        IMPORT kernel_stats_suspend                  ; from portISR.c
        IMPORT kernel_stats_restart                  ; from portISR.c


        PRESERVE8

;-----------------------------------------------------------------------
; void gpOS_interrupt_init( gpOS_partition_t *part, const size_t stack_size);
;-----------------------------------------------------------------------
gpOS_interrupt_init
    ;
    ; save some scratch registers and lr for return
    ;
        stmfd   sp!, {r1-r5,lr}

        mov     r4, r0
        mov     r5, r1

    ;
    ; align memory for stack allocation
    ;
        mov     r1, #TASK_STACKALIGNMENT
        bl      memory_align_p

    ;
    ; align memory size and allocate it
    ;
        add     r1, r5, #(TASK_STACKALIGNMENT - 1)
        and     r1, r1, #~(TASK_STACKALIGNMENT - 1)
        mov     r0, r4
        bl      gpOS_memory_allocate_p

    ;
    ; check if memory was allocated
    ;
        cmp     r0, #0x0
        beq     gpOS_interrupt_init_exit
        mov     r4, r0
    ;
    ; set system stack pointer into kernel
    ;
        add     r1, r0, r5
        ldr     r2, =modeSYS_sp
        str     r1, [r2]
    ;
    ; set system mode stack to some default value
    ;
        mov     r2, r5
        mov     r1, #0x55
    IF :DEF: CLIBS_RETARGETING
        bl      _clibs_memset
    ELSE
        bl      memset
    ENDIF

    ;
    ; write stack name on top of the stack
    ;
        mov     r0, r4
        ldr     r1, =interrupt_stack_name
        mov     r2, #TASK_NAME_SIZE
    IF :DEF: CLIBS_RETARGETING
        bl      _clibs_strncpy
    ELSE
        bl      strncpy
    ENDIF

    ;
    ; init went ok
    ;
        mov     r0, #0x1

gpOS_interrupt_init_exit
        ldmfd   sp!, {r1-r5, lr}          ;restore some scratch registers and return
        bx      lr

;-----------------------------------------------------------------------
; void interrupt_handler( void);
;-----------------------------------------------------------------------
interrupt_handler

    ;
    ;// To replace the __irq directive
    ;
        stmfd   sp!,{r0-r12,lr}

    ;
    ;// IRQ Header
    ;
        bl      interrupt_enter

    ;
    ;// Informs the hw that the ISR has been initiated,
    ;// get:
    ;// - the irq_line (pointed by r0)
    ;// - the handler ptr (pointed by r1)
    ;// - the argument ptr (pointed by r2)
    ;// on IRQ stack
    ;//
        sub     r2, sp, #0x10
        sub     r1, sp, #0xc
        sub     r0, sp, #0x8
        mov     sp, r2
        ldr     r3, =gpOS_bsp_interrupt_dispatch_irq
        mov     lr, pc
        bx      r3

    ;//
    ;// get back;
    ;// - argument ptr in r0
    ;// - handler ptr in r1
    ;// - irq_line in r2
    ;// - dummy value in r3
    ;
        ldmfd   sp!, {r0-r3}

    ;//
    ;// Check if line is safe
    ;//
        teq     r2, #0x80000000
        beq     interrupt_handler_fakeirq

    ;
    ;// ARM IRQ Header
    ;
        mrs     r3, spsr                  ;// save the spsr into r3
        stmfd   sp!, {r2, r3}             ;// save line and msr onto IRQ sp

    ;
    ;// don't enable IRQs, go into system mode and align the stack
    ;
        msr     cpsr_c, #MODE_SYS:OR:I_BIT:OR:F_BIT
        mov     r2, sp
        and     r2, r2, #4
        sub     sp, sp, r2
        stmfd   sp!, {r2, lr}

    IF FREERTOS_PROFILING = {TRUE}
    ;
    ;// Increment number of interrupts occurred
    ;
        ldr     r2, =interrupt_occurred_count       ; r2 = &interrupt_occurred_count
        ldr     r3, [r2]                            ; r3 = *r2
        add     r3, r3, #1                          ; r3 = r3 + 1
        str     r3, [r2]                            ; *r2 = r3
    ENDIF

    ;
    ;// Execute ISR Body routine if any
    ;
        msr     cpsr_c, #MODE_SYS    ;// reenable IRQs, go into system mode
        cmp     r1, #0
        beq     interrupt_handler_footer

    ;
    ;// save argument and ISR vector on stack for later usage
    ;
        stmfd   sp!, {r0-r1}

    IF FREERTOS_IRQ_HOOKS = {TRUE}
    ;
    ;// call interrupt enter hook
    ;
        ldr     r2, =gpOS_bsp_on_interrupt_enter
        mov     lr, pc
        bx      r2
    ENDIF

    ;
    ;// recall argument and ISR vector from stack, but keep them on it for leave hook usage
    ;
        ldmfd   sp, {r0-r1}

    ;
    ;// call ISR vector
    ;

        mov     lr, pc
        bx      r1

    ;
    ;// pop argument and ISR vector from stack
    ;
        ldmfd   sp!, {r0-r1}

    IF FREERTOS_IRQ_HOOKS = {TRUE}
    ;
    ;// call interrupt leave hook
    ;
        ldr     r2, =gpOS_bsp_on_interrupt_leave
        mov     lr, pc
        bx      r2
    ENDIF

    ;
    ;// disable IRQs, go into system mode and align back the stack
    ;
        msr     cpsr_c, #MODE_SYS:OR:I_BIT:OR:F_BIT
        ldmfd   sp!, {r2, lr}
        add     sp, sp, r2

interrupt_handler_footer
    ;
    ;// ARM IRQ Footer
    ;
        msr     cpsr_c, #MODE_IRQ:OR:I_BIT:OR:F_BIT         ;// disable IRQs, move to IRQ mode

        ldmfd   sp!, {r2, r3}         ;// restore spsr in r3 and line in r0 from IRQ sp (line in r2 to align to fake irq)
        msr     spsr_cxsf, r3         ;// restore status register spsr

interrupt_handler_fakeirq
    ;
    ;// get IRQ line back from the stack and clear pending bit in hw
    ;
        mov     r0, r2
        ldr     r2, =gpOS_bsp_interrupt_clear_irq
        mov     lr, pc
        bx      r2

    ;
    ;// IRQ Footer
    ;
        bl      interrupt_leave

    ;
    ;// return from IRQ and interrupt reenabled
    ;
        ldmfd   sp!,{r0-r12,lr}
        subs    pc,lr,#4


;-----------------------------------------------------------------------
; void gpOS_interrupt_lock( void);
;-----------------------------------------------------------------------

gpOS_interrupt_lock
        stmfd   sp!, {r0-r1}          ; save some scratch registers
        mrs     r0, cpsr              ; get cpsr
        ands    r1, r0, #0xf          ; check if USR mode (bit 0-3 should be 0)
        swieq   0x0                   ; if so, call the interrupt disable swi
        orrne   r0, r0, #I_BIT        ; else, disable IRQs (note: this code is
        msrne   cpsr_c, r0            ; not executed when coming back from SWI)

    ;
    ; increment lock count
    ;
        ldr     r0, =interrupt_lock_count
        ldr     r1, [r0]
        add     r1, r1, #0x1
        str     r1, [r0]

        ldmfd   sp!, {r0-r1}          ;restore some scratch registers
        bx      lr

;-----------------------------------------------------------------------
; void gpOS_interrupt_unlock( void);
;-----------------------------------------------------------------------

gpOS_interrupt_unlock
        stmfd   sp!, {r0-r1}          ; save some scratch registers

    ;
    ; check if the interrupt is locked more than once
    ;
        ldr     r1, =interrupt_lock_count
        ldr     r0, [r1]
        subs    r0, r0, #0x1
        strpl   r0, [r1]              ; store counter only if not negative
                                      ; (avoids errors in calling too many unlocks)

        ldmgtfd sp!, {r0-r1}          ; if greater than 0, restore some scratch
        bxgt    lr                    ; registers and return

        mrs     r0, cpsr              ; get cpsr
        ands    r1, r0, #0xf          ; check if USR mode (bit 0-3 should be 0)
        swieq   0x1                   ; if so, call the interrupt enable swi
        bicne   r0, r0, #I_BIT        ; else, enable IRQs (note: this code is
        msrne   cpsr_c, r0            ; not executed when coming back from SWI)

        ldmfd   sp!, {r0-r1}          ; restore some scratch registers
        bx      lr

;-----------------------------------------------------------------------
; void interrupt_enter(void)
;
; pre: IRQ mode
;      either running on IRQ stack or on interruptStack if nested interrupt
;      r0-r12,r14 have been saved
;-----------------------------------------------------------------------

interrupt_enter
    ;
    ; save user SP and LR on IRQ stack. Decrement is not done automatically to avoid
    ; warning from assembler about unpredictable value in sp
    ;
        stmfd   sp, {r13,r14}^          ; *(irq_sp-8) = user_sp, *(irq_sp-4) = user_lr
        nop
        sub     sp, sp, #8              ; irq_sp -= 8

    ;
    ; interrupt_nested_count++
    ;
        ldr     r1, =interrupt_nested_count  ; r1 = &interrupt_nested_count
        ldr     r2, [r1]                  ; r2 = *r1
        add     r2, r2, #1                ; r2 = r2 + 1
        str     r2, [r1]                  ; *r1 = r2

    ;
    ; if (interrupt_nested_count == 1)
    ;   user sp = interruptStack;
    ;
        cmp     r2, #1
        bne     interrupt_enter_isnested

        stmfd   sp!, {r0, lr}

    IF FREERTOS_PROFILING = {TRUE}

    ;
    ; stop counting cpu usage for current task
    ;

        ldr     r0, =kernel_stats_suspend
        mov     lr, pc
        bx      r0

    ENDIF

    IF FREERTOS_IRQ_HOOKS = {TRUE}

    ;
    ; execute interrupt enter hook
    ;

        ldr     r0, =interrupt_enter_hook
        ldr     r0, [r0]
        cmp     r0, #0x0
        movne   lr, pc
        bxne    r0

    ENDIF

        ldmfd   sp!, {r0, lr}

        ldr     r1, =modeSYS_sp         ; r1 = &kernel_SYSmode_sp
        ldr     r1, [r1]                ; r1 = kernel_SYSmode_sp
        stmdb   sp, {r1}                ; *(irq_sp-4) = kernel_SYSmode_sp
        ldmdb   sp, {r13}^              ; user_sp = *(irq_sp-4)

interrupt_enter_isnested
        nop
        bx      lr

;-----------------------------------------------------------------------
; void interrupt_leave(void)
;
; pre:
;      running in system mode,
;      interrupt_enter was called
;      sp value is the same as after interrupt_enter called
;-----------------------------------------------------------------------

interrupt_leave
    ;
    ; restore user SP from IRQ stack. Increment is not done automatically to avoid
    ; warning from assembler about unpredictable value in sp
    ;
        ldmfd   sp, {r13}^          ; *(irq_sp-8) = user_sp, *(irq_sp-4) = user_lr
        nop
        add     sp, sp, #4              ; irq_sp += 8
        ldmfd   sp, {r14}^          ; *(irq_sp-8) = user_sp, *(irq_sp-4) = user_lr
        nop
        add     sp, sp, #4          ; irq_sp += 8

    ;
    ; interrupt_nested_count--
    ;
        ldr     r1, =interrupt_nested_count ; r1 = &interrupt_nested_count
        ldr     r2, [r1]                    ; r2 = *r1
        sub     r2, r2, #1                  ; r2 = r2 - 1
        str     r2, [r1]                    ; *r1 = r2

    ;
    ; if (interrupt_nested_count != 0)
    ;   Continue previous isr, no rescheduling
    ;
        cmp     r2, #0
        bne     interrupt_leave_noreschedule

    ;
    ; execute interrupt leave hook
    ;
        stmfd   sp!, {r0, lr}

    IF FREERTOS_IRQ_HOOKS = {TRUE}

        ldr     r0, =interrupt_leave_hook
        ldr     r0, [r0]
        cmp     r0, #0x0
        movne   lr, pc
        bxne    r0

    ENDIF

    IF FREERTOS_PROFILING = {TRUE}
    ;
    ; start counting cpu usage for current task
    ;
        ldr     r0, =kernel_stats_restart
        mov     lr, pc
        bx      r0

    ENDIF

        ldmfd   sp!, {r0, lr}

    ;
    ; if (!xReschedule_fromISR_required) goto No_Res - nothing to reschedule
    ;
        ldr     r1, =xReschedule_fromISR_required
        ldr     r2, [r1]
        cmp     r2, #1
        bne     interrupt_leave_noreschedule

    ;
    ; If I have to reschedule then the APCS restore registers must be
    ; performed here. Then we will jump into kernel_reschedule_from_interrupt
    ; and we will save there all the regs with SAVE_REGISTERS
    ;
        mov     r0, #0
        ldr     r1, =xReschedule_fromISR_required
        str     r0, [r1]                ; Reset the xReschedule_fromISR_required
        ldmfd   sp!,{r0-r12,r14}        ; restore APCS registers saved by interrupt handler
        sub     r14,r14,#4              ; subtract 4 to point to interrupt instruction
        b       vPortYieldProcessor
    ;
    ;should never come back here....
    ;

interrupt_leave_noreschedule
    ;
    ; return
    ;
        bx      lr

;-----------------------------------------------------------------------
; Data area
;-----------------------------------------------------------------------

        AREA gpOS_interrupt_data, DATA, READWRITE

    ;
    ; variable to force OS reschedule from an interrupt handler
    ;
        EXPORT xReschedule_fromISR_required
xReschedule_fromISR_required   DCD   0x00000000

    ;
    ; interrupt nesting count
    ;
        EXPORT interrupt_nested_count
interrupt_nested_count   DCD   0x00000000

    IF FREERTOS_PROFILING = {TRUE}
    ;
    ; Number of interrupts and context swith occurred
    ;
   ;     EXPORT interrupt_occurred_count
        EXPORT context_switch_count
interrupt_occurred_count DCD   0x00000000
context_switch_count     DCD   0x00000000
    ENDIF

    ;
    ; interrupt lock count
    ;
        EXPORT interrupt_lock_count
interrupt_lock_count     DCD   0x00000000

    ;
    ; stack pointer for SYS mode
    ;
modeSYS_sp          DCD   0x00000000

    IF FREERTOS_IRQ_HOOKS = {TRUE}

    ;
    ; interrupt enter and leave hooks
    ;
        EXPORT interrupt_enter_hook
        EXPORT interrupt_leave_hook
interrupt_enter_hook     DCD   0x00000000
interrupt_leave_hook     DCD   0x00000000

    ENDIF

    ;
    ; name of SYS stack workspace
    ;
interrupt_stack_name     DCB  "OS_SYS_ws"


        END
