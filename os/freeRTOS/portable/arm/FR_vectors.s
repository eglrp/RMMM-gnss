; ************************
; exception Handlers
; ************************
;
; This is the standard vectors table. Each handler could be programmed
; by setting the x_Addr pointer. Note that the only one that
; must be defined outside here is Reset_Handler that could be different for
; each environment.

        ;// OS Exception Vector Table
        AREA vector_table, CODE, READONLY

        ;.code   32
        ;.align  4

        EXPORT  vct_ResetVector
        EXPORT  vct_UndefinedVector
        EXPORT  vct_SWIVector
        EXPORT  vct_PrefetchVector
        EXPORT  vct_AbortVector
        EXPORT  vct_IRQVector
        EXPORT  vct_FIQVector

vct_ResetVector
        ldr     pc, vct_ResetHandlerAddr
vct_UndefinedVector
        ldr     pc, vct_UndefinedHandlerAddr
vct_SWIVector
        ldr     pc, vct_SWIHandlerAddr
vct_PrefetchVector
        ldr     pc, vct_PrefetchHandlerAddr
vct_AbortVector
        ldr     pc, vct_AbortHandlerAddr
vct_DummyVector
        NOP
vct_IRQVector
        ldr     pc, vct_IRQHandlerAddr
vct_FIQVector
        ldr     pc, vct_FIQHandlerAddr

;-----------------------------------------------------------------------
; Data area
;-----------------------------------------------------------------------

        EXPORT  vct_ResetHandlerAddr
        EXPORT  vct_UndefinedHandlerAddr
        EXPORT  vct_SWIHandlerAddr
        EXPORT  vct_PrefetchHandlerAddr
        EXPORT  vct_AbortHandlerAddr
        EXPORT  vct_IRQHandlerAddr
        EXPORT  vct_FIQHandlerAddr

vct_ResetHandlerAddr          DCD     vct_ResetVector        ;// Simple Loop
vct_UndefinedHandlerAddr      DCD     vct_UndefinedVector    ;// Simple Loop
vct_SWIHandlerAddr            DCD     vct_SWIVector          ;// Simple Loop
vct_PrefetchHandlerAddr       DCD     vct_PrefetchVector     ;// Simple Loop
vct_AbortHandlerAddr          DCD     vct_AbortVector        ;// Simple Loop
vct_IRQHandlerAddr            DCD     vct_IRQVector          ;// Simple Loop
vct_FIQHandlerAddr            DCD     vct_FIQVector          ;// Simple Loop

        END
