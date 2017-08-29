@ --- Standard definitions of mode bits and interrupt (I & F) flags in PSRs

.equ  gpOS_MODE_USR             , 0x10
.equ  gpOS_MODE_FIQ             , 0x11
.equ  gpOS_MODE_IRQ             , 0x12
.equ  gpOS_MODE_SVC             , 0x13
.equ  gpOS_MODE_ABT             , 0x17
.equ  gpOS_MODE_UND             , 0x1B
.equ  gpOS_MODE_SYS             , 0x1f @ available on ARM Arch 4 and later

.equ  gpOS_MODE_MASK            , 0x1f @ bit mask for mode

.equ  gpOS_I_BIT                , 0x80 @ when I bit is set, IRQ is disabled
.equ  gpOS_F_BIT                , 0x40 @ when F bit is set, FIQ is disabled
.equ  gpOS_T_BIT                , 0x20

@
@ position onto the stack of the registers after a context switch
@

.equ  gpOS_KRNL_REGS_R0         , 0
.equ  gpOS_KRNL_REGS_R1         , 1
.equ  gpOS_KRNL_REGS_R2         , 2
.equ  gpOS_KRNL_REGS_R3         , 3
.equ  gpOS_KRNL_REGS_R4         , 4
.equ  gpOS_KRNL_REGS_R5         , 5
.equ  gpOS_KRNL_REGS_R6         , 6
.equ  gpOS_KRNL_REGS_R7         , 7
.equ  gpOS_KRNL_REGS_R8         , 8
.equ  gpOS_KRNL_REGS_R9         , 9
.equ  gpOS_KRNL_REGS_R10        , 10
.equ  gpOS_KRNL_REGS_R11        , 11
.equ  gpOS_KRNL_REGS_R12        , 12
.equ  gpOS_KRNL_REGS_R13        , 13
.equ  gpOS_KRNL_REGS_R14        , 14
.equ  gpOS_KRNL_REGS_PC         , 15
.equ  gpOS_KRNL_REGS_CPSR       , 16
.equ  gpOS_KRNL_REGS_DUMMY      , 17

    .if VFP_SUPPORT == TRUE

.equ  gpOS_KRNL_REGS_S0         , 18
.equ  gpOS_KRNL_REGS_S1         , 19
.equ  gpOS_KRNL_REGS_S2         , 20
.equ  gpOS_KRNL_REGS_S3         , 21
.equ  gpOS_KRNL_REGS_S4         , 22
.equ  gpOS_KRNL_REGS_S5         , 23
.equ  gpOS_KRNL_REGS_S6         , 24
.equ  gpOS_KRNL_REGS_S7         , 25
.equ  gpOS_KRNL_REGS_S8         , 26
.equ  gpOS_KRNL_REGS_S9         , 27
.equ  gpOS_KRNL_REGS_S10        , 28
.equ  gpOS_KRNL_REGS_S11        , 29
.equ  gpOS_KRNL_REGS_S12        , 30
.equ  gpOS_KRNL_REGS_S13        , 31
.equ  gpOS_KRNL_REGS_S14        , 32
.equ  gpOS_KRNL_REGS_S15        , 33
.equ  gpOS_KRNL_REGS_S16        , 34
.equ  gpOS_KRNL_REGS_S17        , 35
.equ  gpOS_KRNL_REGS_S18        , 36
.equ  gpOS_KRNL_REGS_S19        , 37
.equ  gpOS_KRNL_REGS_S20        , 38
.equ  gpOS_KRNL_REGS_S21        , 39
.equ  gpOS_KRNL_REGS_S22        , 40
.equ  gpOS_KRNL_REGS_S23        , 41
.equ  gpOS_KRNL_REGS_S24        , 42
.equ  gpOS_KRNL_REGS_S25        , 43
.equ  gpOS_KRNL_REGS_S26        , 44
.equ  gpOS_KRNL_REGS_S27        , 45
.equ  gpOS_KRNL_REGS_S28        , 46
.equ  gpOS_KRNL_REGS_S29        , 47
.equ  gpOS_KRNL_REGS_S30        , 48
.equ  gpOS_KRNL_REGS_S31        , 49
.equ  gpOS_KRNL_REGS_FPEXC      , 50
.equ  gpOS_KRNL_REGS_FPSCR      , 51
.equ  gpOS_KRNL_REGS_FPINST     , 52
.equ  gpOS_KRNL_REGS_FPINST2    , 53
.equ  gpOS_KRNL_REGS_NUMOFREGS  , 54

    .else

.equ  gpOS_KRNL_REGS_NUMOFREGS  , 18

    .endif

.equ  OS20_KRNL_ARM_REGS        , 16

.equ  gpOS_TASK_STACKALIGNMENT  , 8

.equ  gpOS_TASK_NAME_SIZE       , 16
