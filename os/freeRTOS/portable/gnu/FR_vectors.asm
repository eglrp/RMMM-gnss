@ ************************
@ exception Handlers
@ ************************
@
@ This is the standard vectors table. Each handler could be programmed
@ by setting the x_Addr pointer. Note that the only one that
@ must be defined outside here is Reset_Handler that could be different for
@ each environment.

        @// OS Exception Vector Table
        .section .vector_table, "ax"

        .code   32
        .align  4

        .global  vct_ResetVector
        .global  vct_UndefinedVector
        .global  vct_SWIVector
        .global  vct_PrefetchVector
        .global  vct_AbortVector
        .global  vct_IRQVector
        .global  vct_FIQVector

vct_ResetVector:
        ldr     pc, vct_ResetHandlerAddr
vct_UndefinedVector:
        ldr     pc, vct_UndefinedHandlerAddr
vct_SWIVector:
        ldr     pc, vct_SWIHandlerAddr
vct_PrefetchVector:
        ldr     pc, vct_PrefetchHandlerAddr
vct_AbortVector:
        ldr     pc, vct_AbortHandlerAddr
vct_DummyVector:
        NOP
vct_IRQVector:
        ldr     pc, vct_IRQHandlerAddr
vct_FIQVector:
        ldr     pc, vct_FIQHandlerAddr

@-----------------------------------------------------------------------
@ Data area
@-----------------------------------------------------------------------

        .global  vct_ResetHandlerAddr
        .global  vct_UndefinedHandlerAddr
        .global  vct_SWIHandlerAddr
        .global  vct_PrefetchHandlerAddr
        .global  vct_AbortHandlerAddr
        .global  vct_IRQHandlerAddr
        .global  vct_FIQHandlerAddr

vct_ResetHandlerAddr:          .long     vct_ResetVector        @// Simple Loop
vct_UndefinedHandlerAddr:      .long     vct_UndefinedVector    @// Simple Loop
vct_SWIHandlerAddr:            .long     vct_SWIVector          @// Simple Loop
vct_PrefetchHandlerAddr:       .long     vct_PrefetchVector     @// Simple Loop
vct_AbortHandlerAddr:          .long     vct_AbortVector        @// Simple Loop
vct_IRQHandlerAddr:            .long     vct_IRQVector          @// Simple Loop
vct_FIQHandlerAddr:            .long     vct_FIQVector          @// Simple Loop

        .end
