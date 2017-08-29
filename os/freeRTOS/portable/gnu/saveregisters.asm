@------------------------------
@ nasty macro to save registers
@------------------------------


        .macro  RESTORE_VFP_REGISTERS
    @
    @ load the vfp registers from the user stackpointer
    @ pre:
    @     r1 points to user stack region
    @

    .if VFP_SUPPORT == TRUE

        fldmfds r1!, {s0-s31}
        ldmfd r1!, {r3}
        fmxr fpexc, r3
        ldmfd r1!, {r3}
        fmxr fpscr, r3
        ldmfd r1!, {r3}
        fmxr fpinst, r3
        ldmfd r1!, {r3}
        fmxr fpinst2, r3

    .endif

        .endm

        .macro  SAVE_VFP_REGISTERS

    @
    @ save the vfp registers below the user stackpointer
    @ pre:
    @     r1 points to user stack region
    @     r3 is the only spare register available
    @

    .if VFP_SUPPORT == TRUE

        fmrx r3, fpinst2
        stmfd r1!, {r3}
        fmrx r3, fpinst
        stmfd r1!, {r3}
        fmrx r3, fpscr
        stmfd r1!, {r3}
        fmrx r3, fpexc
        stmfd r1!, {r3}
        fstmfds r1!, {s0-s31}

    .endif

        .endm


        .macro  SAVE_REGISTERS
    @
    @ save the user registers below the user stackpointer
    @ pre:
    @     user r0-r3 are stored at sp
    @     r0 can't be used (svc uses as SWI number)
    @     r2 contains saved cpsr
    @ post:
    @     r1 points to user registers
    @

        stmfd   sp, {r13}^            @save user stackpointer into the supervisor stack
        nop
        ldr     r1, [sp,#-4]          @load it into r1

        SAVE_VFP_REGISTERS

        sub     r1, r1, #8            @leave a slot for the cpsr
        stmfd   r1, {r0 - r15}^       @save all user registers into user stack
        str     lr, [r1,#-4]          @update the saved user program counter with the banked link register
        str     r2, [r1]              @put saved cpsr value onto user the stack

        sub     r2, r1, #(KRNL_ARM_REGS*4)    @point r2 at the start of the saved registers

        ldmfd   sp!, {r3-r6}          @copy saved r0-r3 registers into registers
        stmea   r2, {r3-r6}           @and into the user stack
        ldr     r1, =modeUSR_sp       @load address of modeUSR_sp
        str     r2, [r1]              @save it for later
        mov     r1, r2                @and pass it to the handle_system_call function

        .endm


        .macro  RESTORE_REGISTERS
        mrs     r0, CPSR
        ldr     r1, =modeSVC_sp
        ldr     sp, [r1]

      @
      @ r1 = modeUSR_sp
      @

        ldr     r1, =modeUSR_sp
        ldr     r0, [r1]              @r1 contains pointer to registers, r0[0] = register 0 etc

      @
      @ restore VFP registers if VFP is enabled
      @
      .if VFP_SUPPORT == TRUE

        add     r1, r0, #(KRNL_REGS_S0*4)         @ r1 now points to VFP registers if any
        RESTORE_VFP_REGISTERS

      .endif

      @
      @ spsr = modeUSR_sp[psr]
      @
        ldr     r1, [r0,#KRNL_REGS_CPSR*4]       @get cpsr value
        msr     spsr_cf, r1                      @and restore into spsr
      @
      @ lr = modeUSR_sp[pc]
      @
        ldr     lr, [r0,#KRNL_REGS_PC*4]
      @
      @ r{0..14} = modeUSR_sp[0..14]
      @
        ldmfd   r0, {r0 - r14}^
        nop

        .endm
