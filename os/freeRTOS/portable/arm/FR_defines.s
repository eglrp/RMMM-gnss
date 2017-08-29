; --- Standard definitions of mode bits and interrupt (I & F) flags in PSRs

MODE_USR             EQU 0x10
MODE_FIQ             EQU 0x11
MODE_IRQ             EQU 0x12
MODE_SVC             EQU 0x13
MODE_ABT             EQU 0x17
MODE_UND             EQU 0x1B
MODE_SYS             EQU 0x1f ; available on ARM Arch 4 and later

MODE_MASK            EQU 0x1f ; bit mask for mode

I_BIT                EQU 0x80 ; when I bit is set, IRQ is disabled
F_BIT                EQU 0x40 ; when F bit is set, FIQ is disabled
T_BIT                EQU 0x20

PORT_YIELD_SWI       EQU 0x4

;
; position onto the stack of the registers after a context switch
;

KRNL_REGS_R0         EQU 0
KRNL_REGS_R1         EQU 1
KRNL_REGS_R2         EQU 2
KRNL_REGS_R3         EQU 3
KRNL_REGS_R4         EQU 4
KRNL_REGS_R5         EQU 5
KRNL_REGS_R6         EQU 6
KRNL_REGS_R7         EQU 7
KRNL_REGS_R8         EQU 8
KRNL_REGS_R9         EQU 9
KRNL_REGS_R10        EQU 10
KRNL_REGS_R11        EQU 11
KRNL_REGS_R12        EQU 12
KRNL_REGS_R13        EQU 13
KRNL_REGS_R14        EQU 14
KRNL_REGS_PC         EQU 15
KRNL_REGS_CPSR       EQU 16
KRNL_REGS_DUMMY      EQU 17

    IF {FPU} <> "SoftVFP"

KRNL_REGS_S0         EQU 18
KRNL_REGS_S1         EQU 19
KRNL_REGS_S2         EQU 20
KRNL_REGS_S3         EQU 21
KRNL_REGS_S4         EQU 22
KRNL_REGS_S5         EQU 23
KRNL_REGS_S6         EQU 24
KRNL_REGS_S7         EQU 25
KRNL_REGS_S8         EQU 26
KRNL_REGS_S9         EQU 27
KRNL_REGS_S10        EQU 28
KRNL_REGS_S11        EQU 29
KRNL_REGS_S12        EQU 30
KRNL_REGS_S13        EQU 31
KRNL_REGS_S14        EQU 32
KRNL_REGS_S15        EQU 33
KRNL_REGS_S16        EQU 34
KRNL_REGS_S17        EQU 35
KRNL_REGS_S18        EQU 36
KRNL_REGS_S19        EQU 37
KRNL_REGS_S20        EQU 38
KRNL_REGS_S21        EQU 39
KRNL_REGS_S22        EQU 40
KRNL_REGS_S23        EQU 41
KRNL_REGS_S24        EQU 42
KRNL_REGS_S25        EQU 43
KRNL_REGS_S26        EQU 44
KRNL_REGS_S27        EQU 45
KRNL_REGS_S28        EQU 46
KRNL_REGS_S29        EQU 47
KRNL_REGS_S30        EQU 48
KRNL_REGS_S31        EQU 49
KRNL_REGS_FPEXC      EQU 50
KRNL_REGS_FPSCR      EQU 51
KRNL_REGS_FPINST     EQU 52
KRNL_REGS_FPINST2    EQU 53
KRNL_REGS_NUMOFREGS  EQU 54

    ELSE

KRNL_REGS_NUMOFREGS  EQU 18

    ENDIF

KRNL_ARM_REGS        EQU 16

TASK_STACKALIGNMENT  EQU 8

TASK_NAME_SIZE       EQU 16

       END
