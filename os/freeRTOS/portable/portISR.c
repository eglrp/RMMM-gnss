/*lint --e{9059}  : C comment contains C++ comment [MISRA 2012 Rule 3.1, required]*/
/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/*-----------------------------------------------------------
 * Components that can be compiled to either ARM or THUMB mode are
 * contained in port.c  The ISR routines, which can only be compiled
 * to ARM mode, are contained in this file.
 *----------------------------------------------------------*/

/*
	Changes from V3.2.4

	+ The assembler statements are now included in a single asm block rather
	  than each line having its own asm block.
*/


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "FRi.h"
#include "FR_svci.h"
#include "FR_utils.h"

#if defined( FREERTOS_PROFILING)
#include "gpOS.h"
#endif

/* Constants required to handle critical sections. */
#define portNO_CRITICAL_NESTING		( ( uint32_t ) 0UL )
volatile uint32_t ulCriticalNesting = 0UL;
extern uint32_t interrupt_lock_count;

#if defined ( __ARMCC_VERSION)
extern void vPortEnterCriticalArm(void);
extern void vPortExitCriticalArm(void);
#endif

#if defined( __GNUC__)
/*-----------------------------------------------------------*/

/* ISR to handle manual context switches (from a call to taskYIELD()). */
void vPortYieldProcessor( void ) __attribute__((interrupt("SWI"), naked));

/*
 * The scheduler can only be started from ARM mode, hence the inclusion of this
 * function here.
 */
void vPortISRStartFirstTask( void );
/*-----------------------------------------------------------*/

void vPortISRStartFirstTask( void )
{
	/* Simply start the scheduler.  This is included here as it can only be
	called from ARM mode. */
	portRESTORE_CONTEXT();
}
/*-----------------------------------------------------------*/

/*
 * Called by portYIELD() or taskYIELD() to manually force a context switch.
 *
 * When a context switch is performed from the task level the saved task
 * context is made to look as if it occurred from within the tick ISR.  This
 * way the same restore context function can be used when restoring the context
 * saved from the ISR or that saved from a call to vPortYieldProcessor.
 */
void vPortYieldProcessor( void )
{
	/* Within an IRQ ISR the link register has an offset from the true return
	address, but an SWI ISR does not.  Add the offset manually so the same
	ISR return code can be used in both cases. */

  asm volatile(
    "add   LR, LR, #4     ;"
  );

	/* Perform the context switch.  First save the context of the current task. */
	portSAVE_CONTEXT();

	/* Find the highest priority task that is ready to run. */
	vTaskSwitchContext();

	/* Restore the context of the new task. */
	portRESTORE_CONTEXT();
}
/*-----------------------------------------------------------*/

/*
 * The ISR used for the scheduler tick depends on whether the cooperative or
 * the preemptive scheduler is being used.
 */

#if configUSE_PREEMPTION == 0

	/* The cooperative scheduler requires a normal IRQ service routine to
	simply increment the system tick. */
	void vNonPreemptiveTick( void ) __attribute__ ((interrupt ("IRQ")));
	void vNonPreemptiveTick( void )
	{
		xTaskIncrementTick();
	}

#else  /* else preemption is turned on */

	/* The preemptive scheduler is defined as "naked" as the full context is
	saved on entry as part of the context switch. */
	void vPreemptiveTick( void ) __attribute__((naked));
	void vPreemptiveTick( void )
	{
		/* Save the context of the interrupted task. */
		portSAVE_CONTEXT();

		/* WARNING - Do not use local (stack) variables here.  Use globals
					 if you must! */

		/* Increment the RTOS tick count, then look for the highest priority
		task that is ready to run. */
		if( xTaskIncrementTick() != pdFALSE )
		{
			vTaskSwitchContext();
		}

		/* Restore the context of the new task. */
		portRESTORE_CONTEXT();
	}

#endif
/*-----------------------------------------------------------*/

/*
 * The interrupt management utilities can only be called from ARM mode.  When
 * THUMB_INTERWORK is defined the utilities are defined as functions here to
 * ensure a switch to ARM mode.  When THUMB_INTERWORK is not defined then
 * the utilities are defined as macros in portmacro.h - as per other ports.
 */
#ifdef THUMB_INTERWORK

	void vPortDisableInterruptsFromThumb( void ) __attribute__ ((naked));
	void vPortEnableInterruptsFromThumb( void ) __attribute__ ((naked));

	void vPortDisableInterruptsFromThumb( void )
	{
		asm volatile (
        "stmfd   sp!, {r0-r1}   \n\t"
        "mrs     r0, cpsr   \n\t"
        "ands    r1, r0, #0xf   \n\t"
        "swieq   0x0    \n\t"
        "orrne   r0, r0, #0x80   \n\t"
        "msrne   cpsr_c, r0   \n\t"
        "ldmfd   sp!, {r0-r1}   \n\t"
        "bx      lr   \n\t"
    );
	}

	void vPortEnableInterruptsFromThumb( void )
	{
		asm volatile (
        "stmfd   sp!, {r0-r1}   \n\t"
        "mrs     r0, cpsr   \n\t"
        "ands    r1, r0, #0xf   \n\t"
        "swieq   0x1    \n\t"
        "bicne   r0, r0, #0x80   \n\t"
        "msrne   cpsr_c, r0   \n\t"
        "ldmfd   sp!, {r0-r1}   \n\t"
        "bx      lr   \n\t"
    );
	}

#endif /* THUMB_INTERWORK */
#endif /* __GNUC__ */

/* The code generated by the GCC compiler uses the stack in different ways at
different optimisation levels.  The interrupt flags can therefore not always
be saved to the stack.  Instead the critical section nesting level is stored
in a variable, which is then saved as part of the stack context. */
void vPortEnterCritical( void )
{
/* Disable interrupts as per portDISABLE_INTERRUPTS(); 							*/
#if defined( __GNUC__)
	asm volatile (
        "stmfd   sp!, {r0-r1}   \n\t"
        "mrs     r0, cpsr   \n\t"
        "ands    r1, r0, #0xf   \n\t"
        "swieq   0x0    \n\t"
        "orrne   r0, r0, #0x80   \n\t"
        "msrne   cpsr_c, r0   \n\t"
        "ldmfd   sp!, {r0-r1}   \n\t"
  );
#else
   vPortEnterCriticalArm();
#endif

	/* Now interrupts are disabled ulCriticalNesting can be accessed
	directly.  Increment ulCriticalNesting to keep a count of how many times
	portENTER_CRITICAL() has been called. */
	ulCriticalNesting++;
}

void vPortExitCritical( void )
{
	if( ulCriticalNesting > portNO_CRITICAL_NESTING )
	{
		/* Decrement the nesting count as we are leaving a critical section. */
		ulCriticalNesting--;

		/* If the nesting level has reached zero AND if the gpOS_interrupt_unlock()
		   has not been previously called then interrupts should be re-enabled. */

    if((ulCriticalNesting == portNO_CRITICAL_NESTING) && (interrupt_lock_count == 0UL))
    {
			/* Enable interrupts as per portEXIT_CRITICAL().				*/
#if defined( __GNUC__)
			asm volatile (
        "stmfd   sp!, {r0-r1}   \n\t"
        "mrs     r0, cpsr   \n\t"
        "ands    r1, r0, #0xf   \n\t"
        "swieq   0x1    \n\t"
        "bicne   r0, r0, #0x80   \n\t"
        "msrne   cpsr_c, r0   \n\t"
        "ldmfd   sp!, {r0-r1}   \n\t"
      );
#else
      vPortExitCriticalArm();
#endif
    }
	}
}

#if defined( FREERTOS_PROFILING)
/********************************************//**
 * \brief   Suspend kernel statistics collecting of
 *          current task during interrupt execution
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
volatile uint32_t itTimePeriodCumul;
volatile uint32_t itTimeCumul;
volatile uint32_t itTimeStart;

void kernel_stats_suspend( void);
void kernel_stats_restart( void);

void kernel_stats_suspend( void)
{
    itTimeStart = gpOS_time_now();
}

/********************************************//**
 * \brief   Restart kernel statistics collecting of
 *          current task during interrupt execution
 *
 * \param void
 * \return void
 *
 ***********************************************/
void kernel_stats_restart( void)
{
    int32_t delta = (int32_t)(gpOS_time_now() - itTimeStart);
    itTimeCumul += delta;
    itTimePeriodCumul += delta;
}
#endif  // FREERTOS_PROFILING


