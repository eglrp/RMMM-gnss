;
; crt0_FR.s
;
; Implements hooks for ARM RVCT libraries for
; - code/data relocation
; - stack/heap initialization
;
;

        GET FR_defines.s
        GET mapping_sta8090.s

        PRESERVE8

        AREA reset_vector_area, CODE, READONLY

        IMPORT  gpOS_memory_init
        IMPORT  gpOS_memory_partition_extend_heap
        IMPORT  gpOS_memory_lock
        IMPORT  gpOS_memory_unlock

        IMPORT     LLD_ARM946_Reset


        IMPORT     platform_usb_sense_init
        IMPORT     gpOS_bsp_reset_handler
        IMPORT     platform_setup
        IMPORT     __main

        IMPORT  os_root_task_stack_size           ; from main.c
        IMPORT  ||Image$$OS_STACK_AREA$$Base||    ; from scatter file

        EXPORT     reset_vector
        EXPORT     reset_handler
        EXPORT     _sys_exit

reset_vector
        ldr     pc, reset_handler_ptr

reset_handler_ptr      DCD       reset_handler

        AREA reset_handler_area, CODE, READONLY

reset_handler
    ;
    ; --- Reset ARM status to safe condition
    ;
        msr     cpsr_c, #MODE_SVC:OR:I_BIT:OR:F_BIT         ;// disable IRQs, move to SVC mode

    ;
    ; --- Reset USB status to prevent host enumeration
    ;
        ldr     r1,=platform_usb_sense_init
        mov     lr, pc
        blx     r1

    ;
    ; Reset micro
    ;
        ldr     r1,=LLD_ARM946_Reset
        mov     lr, pc
        blx     r1

    ;
    ; --- Configure data TCM to have some stack
    ;
        ldr     r0, =(DTCM_START_ADDR:OR:0x12)
        mcr     p15,0,r0,c9,c1,0

        mrc     p15,0,r0,c1,c0,0
        mov     r2,#0x00010000
        orr     r0,r0,r2
        mcr     p15,0,r0,c1,c0,0

    ;
    ; --- Provide stack for memory setup step
    ;
        ldr     r0, =(DTCM_START_ADDR+DTCM_SIZE_MIN)
        add     r0, r0, #0x4
        and     r0, r0, #0xFFFFFFF8
        mov     sp, r0

    ;
    ; --- Execute specific platform setup
    ;
        blx     platform_setup

    ;
    ; --- Execute specific bsp reset
    ;
        blx     gpOS_bsp_reset_handler

    ;
    ; Main program invocation.
    ;

        b       __main


;-----------------------------------------------------------------------
; For RVCT 3.1:
; void __rt_stackheap_init();
; On exit:
; - r0 = heap base
; - r1 = heap limit
; - sp = stack base
; - sl = stack limit
;
; For RVCT 4.0+
; void __user_setup_stackheap();
; On exit:
; - r0 = heap base
; - r2 = heap limit
; - sp = stack base
; - r3 = stack limit ( = sl)
;
;-----------------------------------------------------------------------

    ; Initialisation function. This routine isn't required to
    ; preserve any registers, although obviously it shouldn't
    ; lose LR because it won't know where to return to.

  IF {ARMASM_VERSION} <= 400000
        EXPORT  __rt_stackheap_init
__rt_stackheap_init
  ELSE
        EXPORT  __user_setup_stackheap
__user_setup_stackheap
  ENDIF

        ldr     r0, =||Image$$OS_STACK_AREA$$Base||
        add     r0, r0, #0x4
        and     r0, r0, #0xFFFFFFF8
        mov     sp, r0

    ;
    ; Reserve space for minimal heap if enabled
    ;
        mov     r1, sp
        sub     sp, sp, #0x10
        mov     r0, sp


    ;
    ; Reserve stack size for root task from OS heap region
    ;

        ldr     r4, =os_root_task_stack_size
        ldr     r3, [r4]
        sub     sl, sp, r3

    ;
    ; Clear root task stack
    ;

        mov     r4, sl
        ldr     r3,=0xAAAAAAAA

__os_stackheapinit_stack_cycle
        str     r3, [r4]
        add     r4, r4, #0x4
        cmp     r4, sp
        bne     __os_stackheapinit_stack_cycle

    ;
    ; Reserve space on root stack for registers saving
    ;

        sub     sp, sp, #(KRNL_REGS_NUMOFREGS << 2)

    ;
    ; Initialize memory module for heap configuration (needed for static C++ objects)
    ;
        stmfd   sp!, {r0, lr}

        ldr     r0, =gpOS_memory_init
        mov     lr, pc
        bx      r0
        ldmfd   sp!, {r0, lr}

    ;
    ; For __user_setup_stackheap, setup r2 and r3 properly
    ;
  IF {ARMASM_VERSION} > 400000
        mov     r2, r1
        mov     r3, sl
  ENDIF

        bx      lr

;-----------------------------------------------------------------------
; void __rt_stack_overflow();
;-----------------------------------------------------------------------
    ; The stack overflow function. This is called from the
    ; entry sequence of any routine that fails a stack limit
    ; check. If stack-limit checking is not enabled, this
    ; function is not needed at all.

    ; If the memory model supports non-fatal stack overflows,
    ; this function will need to return. It should return with
    ; _all_ registers preserved as they were on entry (except
    ; SP and SL, which it is allowed to modify to preserve
    ; stack validity). It should not try to return to LR, but
    ; should return by branching to __rt_stack_overflow_return.

    ; On entry, IP (r12) contains the value to which the
    ; calling function would like to drop SP.

        EXPORT  __rt_stack_overflow

__rt_stack_overflow
    ;
    ; Stop execution immediately.
    ;
        mov     a1, #100
        b       _sys_exit

;-----------------------------------------------------------------------
; void __rt_heap_extend();
;-----------------------------------------------------------------------
    ; The user heap extend function. This is called using the
    ; ordinary ATPCS calling convention.
    ;
    ; On input: a1 is the minimum size of memory required
    ;              (guaranteed to be a multiple of 4)
    ;           a2 is a pointer to a word of memory, W, in which
    ;              to store the address of the block
    ;
    ; On exit:  a1 is size of returned block
    ;           W  contains base address of returned block
    ;      Or:  a1 is zero
    ;           W  has undefined contents
    ;
    ; Refuse all requests for extra heap space.
    ;

        EXPORT  __rt_heap_extend

__rt_heap_extend
        stmfd   sp!, {r2, lr}
        mov     r2, r0

    ;
    ; Ask for memory to memory management
    ;
        ldr     r2, =gpOS_memory_partition_extend_heap
        mov     lr, pc
        bx      r2

        ldmfd   sp!, {r2, lr}
        bx      lr

;-----------------------------------------------------------------------
; void _sys_exit();
;-----------------------------------------------------------------------
       EXPORT   _sys_exit
_sys_exit
        b       _sys_exit

;-----------------------------------------------------------------------
; void _ttywrch(int ch)
;-----------------------------------------------------------------------
        EXPORT  _ttywrch
_ttywrch
        bx      lr

        END
