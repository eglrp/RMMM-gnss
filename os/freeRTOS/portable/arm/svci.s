        GET FRi.s
        GET FR_defines.s
        GET saveregisters.s

        AREA    svc_exception, CODE, READONLY

        EXPORT svc_SWI_handler

        IMPORT xSVC_handle_user_system_call        ;from port.c
        IMPORT vPortYieldProcessor                 ;from portasm.s

        PRESERVE8

;-----------------------------------------------------------------------
; void svc_SWI_handler(unsigned char *superstacktop, unsigned char *irqstacktop);
;-----------------------------------------------------------------------

svc_SWI_handler
    ;
    ; reset supervisor stack
    ;
        ldr     sp, =modeSVC_sp
        ldr     sp, [sp]
        stmfd   sp!, {r0-r3}                    ;save some scratch registers

    ;
    ; .include the SWI operand. This depends on whether it was called in normal or thumb mode
    ;
        mrs     r2, spsr
        ands    r0, r2, #T_BIT
        ldreq   r0, [lr,#-4]                    ;load the SWI instruction into r0
        biceq   r0, r0, #0xff000000             ;mask off the top 8 bits to give SWI instruction number
        ldrneh  r0, [lr,#-2]
        bicne   r0, r0, #0xffffff00             ;mask off the top 8 bits to give SWI instruction number

    ;
    ; deal with interrupt disable/enable directly (using the funky conditional instructions)
    ;
        subs    r3, r0, #0x1
        orrmi   r2, r2, #I_BIT
        biceq   r2, r2, #I_BIT
        msrle   spsr_cf, r2
        ldmlefd sp!, {r0-r3}                    ;put saved registers back
        movles  pc, lr                          ;return from exception

    ;
    ; Check if "SVC_PORT_YIELD" is called, calls directly the vPortYieldProcessor
    ;
        subs    r3, r0, #PORT_YIELD_SWI
        ldmeqfd sp!, {r0-r3}                    ;put saved registers back
        beq     vPortYieldProcessor

    ; else ...
        SAVE_REGISTERS
        bl      xSVC_handle_user_system_call    ; xSVC_handle_user_system_call( swi, regs[numregs])
        RESTORE_REGISTERS

    ; move into user mode with PC = lr
    ;
        movs    pc, lr

;-----------------------------------------------------------------------
; Data area
;-----------------------------------------------------------------------

        AREA svc_exception_data, DATA, READWRITE

    ;
    ; stack pointer for SVC mode
    ;
        EXPORT modeSVC_sp
        EXPORT modeUSR_sp
        EXPORT modeIRQ_sp

modeSVC_sp               DCD 0x00000000  ;contains supervisor stack pointer
modeUSR_sp               DCD 0x00000000
modeIRQ_sp               DCD 0x00000000

        END
