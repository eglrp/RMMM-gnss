@ --- Standard definitions of mode bits and interrupt (I & F) flags in PSRs

.equ  MODE_USR             , 0x10
.equ  MODE_FIQ             , 0x11
.equ  MODE_IRQ             , 0x12
.equ  MODE_SVC             , 0x13
.equ  MODE_ABT             , 0x17
.equ  MODE_UND             , 0x1B
.equ  MODE_SYS             , 0x1f @ available on ARM Arch 4 and later

.equ  MODE_MASK            , 0x1f @ bit mask for mode

.equ  I_BIT                , 0x80 @ when I bit is set, IRQ is disabled
.equ  F_BIT                , 0x40 @ when F bit is set, FIQ is disabled
.equ  T_BIT                , 0x20

.equ  PORT_YIELD_SWI       , 0x4

@
@ position onto the stack of the registers after a context switch
@

.equ  KRNL_REGS_R0         , 0
.equ  KRNL_REGS_R1         , 1
.equ  KRNL_REGS_R2         , 2
.equ  KRNL_REGS_R3         , 3
.equ  KRNL_REGS_R4         , 4
.equ  KRNL_REGS_R5         , 5
.equ  KRNL_REGS_R6         , 6
.equ  KRNL_REGS_R7         , 7
.equ  KRNL_REGS_R8         , 8
.equ  KRNL_REGS_R9         , 9
.equ  KRNL_REGS_R10        , 10
.equ  KRNL_REGS_R11        , 11
.equ  KRNL_REGS_R12        , 12
.equ  KRNL_REGS_R13        , 13
.equ  KRNL_REGS_R14        , 14
.equ  KRNL_REGS_PC         , 15
.equ  KRNL_REGS_CPSR       , 16
.equ  KRNL_REGS_DUMMY      , 17

    .if VFP_SUPPORT == TRUE

.equ  KRNL_REGS_S0         , 18
.equ  KRNL_REGS_S1         , 19
.equ  KRNL_REGS_S2         , 20
.equ  KRNL_REGS_S3         , 21
.equ  KRNL_REGS_S4         , 22
.equ  KRNL_REGS_S5         , 23
.equ  KRNL_REGS_S6         , 24
.equ  KRNL_REGS_S7         , 25
.equ  KRNL_REGS_S8         , 26
.equ  KRNL_REGS_S9         , 27
.equ  KRNL_REGS_S10        , 28
.equ  KRNL_REGS_S11        , 29
.equ  KRNL_REGS_S12        , 30
.equ  KRNL_REGS_S13        , 31
.equ  KRNL_REGS_S14        , 32
.equ  KRNL_REGS_S15        , 33
.equ  KRNL_REGS_S16        , 34
.equ  KRNL_REGS_S17        , 35
.equ  KRNL_REGS_S18        , 36
.equ  KRNL_REGS_S19        , 37
.equ  KRNL_REGS_S20        , 38
.equ  KRNL_REGS_S21        , 39
.equ  KRNL_REGS_S22        , 40
.equ  KRNL_REGS_S23        , 41
.equ  KRNL_REGS_S24        , 42
.equ  KRNL_REGS_S25        , 43
.equ  KRNL_REGS_S26        , 44
.equ  KRNL_REGS_S27        , 45
.equ  KRNL_REGS_S28        , 46
.equ  KRNL_REGS_S29        , 47
.equ  KRNL_REGS_S30        , 48
.equ  KRNL_REGS_S31        , 49
.equ  KRNL_REGS_FPEXC      , 50
.equ  KRNL_REGS_FPSCR      , 51
.equ  KRNL_REGS_FPINST     , 52
.equ  KRNL_REGS_FPINST2    , 53
.equ  KRNL_REGS_NUMOFREGS  , 54

    .else

.equ  KRNL_REGS_NUMOFREGS  , 18

    .endif

.equ  KRNL_ARM_REGS        , 16

.equ  TASK_STACKALIGNMENT  , 8

.equ  TASK_NAME_SIZE       , 16
