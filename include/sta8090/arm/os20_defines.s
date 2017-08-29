; --- Standard definitions of mode bits and interrupt (I & F) flags in PSRs

gpOS_MODE_USR             EQU     0x10
gpOS_MODE_FIQ             EQU     0x11
gpOS_MODE_IRQ             EQU     0x12
gpOS_MODE_SVC             EQU     0x13
gpOS_MODE_ABT             EQU     0x17
gpOS_MODE_UND             EQU     0x1B
gpOS_MODE_SYS             EQU     0x1f ; available on ARM Arch 4 and later

gpOS_MODE_MASK            EQU     0x1f ; bit mask for mode

gpOS_I_BIT                EQU     0x80 ; when I bit is set, IRQ is disabled
gpOS_F_BIT                EQU     0x40 ; when F bit is set, FIQ is disabled
gpOS_T_BIT                EQU     0x20

;
; position onto the stack of the registers after a context switch
;

gpOS_KRNL_REGS_R0         EQU     0
gpOS_KRNL_REGS_R1         EQU     1
gpOS_KRNL_REGS_R2         EQU     2
gpOS_KRNL_REGS_R3         EQU     3
gpOS_KRNL_REGS_R4         EQU     4
gpOS_KRNL_REGS_R5         EQU     5
gpOS_KRNL_REGS_R6         EQU     6
gpOS_KRNL_REGS_R7         EQU     7
gpOS_KRNL_REGS_R8         EQU     8
gpOS_KRNL_REGS_R9         EQU     9
gpOS_KRNL_REGS_R10        EQU     10
gpOS_KRNL_REGS_R11        EQU     11
gpOS_KRNL_REGS_R12        EQU     12
gpOS_KRNL_REGS_R13        EQU     13
gpOS_KRNL_REGS_R14        EQU     14
gpOS_KRNL_REGS_PC         EQU     15
gpOS_KRNL_REGS_CPSR       EQU     16
gpOS_KRNL_REGS_DUMMY      EQU     17

    IF {FPU} <> "SoftVFP"

gpOS_KRNL_REGS_S0         EQU     18
gpOS_KRNL_REGS_S1         EQU     19
gpOS_KRNL_REGS_S2         EQU     20
gpOS_KRNL_REGS_S3         EQU     21
gpOS_KRNL_REGS_S4         EQU     22
gpOS_KRNL_REGS_S5         EQU     23
gpOS_KRNL_REGS_S6         EQU     24
gpOS_KRNL_REGS_S7         EQU     25
gpOS_KRNL_REGS_S8         EQU     26
gpOS_KRNL_REGS_S9         EQU     27
gpOS_KRNL_REGS_S10        EQU     28
gpOS_KRNL_REGS_S11        EQU     29
gpOS_KRNL_REGS_S12        EQU     30
gpOS_KRNL_REGS_S13        EQU     31
gpOS_KRNL_REGS_S14        EQU     32
gpOS_KRNL_REGS_S15        EQU     33
gpOS_KRNL_REGS_S16        EQU     34
gpOS_KRNL_REGS_S17        EQU     35
gpOS_KRNL_REGS_S18        EQU     36
gpOS_KRNL_REGS_S19        EQU     37
gpOS_KRNL_REGS_S20        EQU     38
gpOS_KRNL_REGS_S21        EQU     39
gpOS_KRNL_REGS_S22        EQU     40
gpOS_KRNL_REGS_S23        EQU     41
gpOS_KRNL_REGS_S24        EQU     42
gpOS_KRNL_REGS_S25        EQU     43
gpOS_KRNL_REGS_S26        EQU     44
gpOS_KRNL_REGS_S27        EQU     45
gpOS_KRNL_REGS_S28        EQU     46
gpOS_KRNL_REGS_S29        EQU     47
gpOS_KRNL_REGS_S30        EQU     48
gpOS_KRNL_REGS_S31        EQU     49
gpOS_KRNL_REGS_FPEXC      EQU     50
gpOS_KRNL_REGS_FPSCR      EQU     51
gpOS_KRNL_REGS_FPINST     EQU     52
gpOS_KRNL_REGS_FPINST2    EQU     53
gpOS_KRNL_REGS_NUMOFREGS  EQU     54

    ELSE

gpOS_KRNL_REGS_NUMOFREGS  EQU     18

    ENDIF

OS20_KRNL_ARM_REGS        EQU     16

gpOS_TASK_STACKALIGNMENT  EQU     8

gpOS_TASK_NAME_SIZE       EQU     16

        END
