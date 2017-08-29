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
 * Implementation of functions defined in portable.h for the Atmel AT91R40008
 * port.
 *
 * Components that can be compiled to either ARM or THUMB mode are
 * contained in this file.  The ISR routines, which can only be compiled
 * to ARM mode are contained in portISR.c.
 *----------------------------------------------------------*/

/* Standard includes. */
#include <stdlib.h>

#include "clibs.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "portmacro.h"
#include "FRi.h"
#include "FR_memory.h"
#include "FR_svci.h"
#include "FR_timei.h"
#include "gpOS_time.h"

#include "svc_mcu.h"

/* Constants required to setup the task context. */
#define portINITIAL_SPSR				( ( StackType_t ) 0x50 ) /* System mode, ARM mode, interrupts disabled. */
#define portTHUMB_MODE_BIT				( ( StackType_t ) 0x20 )
#define portINSTRUCTION_SIZE			( ( StackType_t ) 4 )
#define portNO_CRITICAL_SECTION_NESTING	( ( StackType_t ) 0 )
#define portTICK_PRIORITY_6				( 6 )
/*-----------------------------------------------------------*/
/* Flag set from the tick interrupt to allow the sleep processing to know if
sleep mode was exited because of an AST interrupt or a different interrupt. */
boolean_t ulTickFlag = pdFALSE;

/* Setup the timer to generate the tick interrupts. */
static void prvSetupTimerInterrupt( void );

/*
 * The scheduler can only be started from ARM mode, so...
 */
#if defined( __GNUC__)
/*
 * ...vPortISRStartFirstSTask() is defined in portISR.c.
 */
extern void vPortISRStartFirstTask( void );

#else
/*
 * ...vPortStartFirstSTask() is defined in portasm.s.
 */
extern void vPortStartFirstTask( void );
#endif

/*-----------------------------------------------------------*/

/*
 * Initialise the stack of a task to look exactly as if a call to
 * portSAVE_CONTEXT had been called.
 *
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
StackType_t *pxOriginalTOS;

	pxOriginalTOS = pxTopOfStack;

	/* To ensure asserts in tasks.c don't fail, although in this case the assert
	is not really required. */
	pxTopOfStack--;

	/* Setup the initial stack of the task.  The stack is set exactly as
	expected by the portRESTORE_CONTEXT() macro. */

	/* First on the stack is the return address - which in this case is the
	start of the task.  The offset is added to make the return address appear
	as it would within an IRQ ISR. */
	*pxTopOfStack = ( StackType_t ) pxCode + portINSTRUCTION_SIZE;
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xaaaaaaaa;	/* R14 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) pxOriginalTOS; /* Stack used when task starts goes in R13. */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x12121212;	/* R12 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x11111111;	/* R11 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x10101010;	/* R10 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x09090909;	/* R9 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x08080808;	/* R8 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x07070707;	/* R7 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x06060606;	/* R6 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x05050505;	/* R5 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x04040404;	/* R4 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x03030303;	/* R3 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x02020202;	/* R2 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x01010101;	/* R1 */
	pxTopOfStack--;

	/* When the task starts is will expect to find the function parameter in
	R0. */
	*pxTopOfStack = ( StackType_t ) pvParameters; /* R0 */
	pxTopOfStack--;

	/* The last thing onto the stack is the status register, which is set for
	system mode, with interrupts enabled. */
	*pxTopOfStack = ( StackType_t ) portINITIAL_SPSR;

	#ifdef THUMB_INTERWORK
	{
		/* We want the task to start in thumb mode. */
		*pxTopOfStack |= portTHUMB_MODE_BIT;
	}
	#endif

	pxTopOfStack--;

	/* Some optimisation levels use the stack differently to others.  This
	means the interrupt flags cannot always be stored on the stack and will
	instead be stored in a variable, which is then saved as part of the
	tasks context. */
	*pxTopOfStack = portNO_CRITICAL_SECTION_NESTING;

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	prvSetupTimerInterrupt();

	/* Start the first task. */
#if defined( __GNUC__)
	vPortISRStartFirstTask();
#else
  vPortStartFirstTask();
#endif

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the ARM port will require this function as there
	is nothing to return to.  */
}
/*-----------------------------------------------------------*/

/*
 * Setup the tick timer to generate the tick interrupts at the required frequency.
 */
static void prvSetupTimerInterrupt( void )
{
	/* Setup the AIC for PIT interrupts.  The interrupt routine chosen depends
	on whether the preemptive or cooperative scheduler is being used. */
	#if configUSE_PREEMPTION == 0

		extern void ( vNonPreemptiveTick ) ( void );
		//AT91F_AIC_ConfigureIt( AT91C_ID_SYS, AT91C_AIC_PRIOR_HIGHEST, portINT_LEVEL_SENSITIVE, ( void (*)(void) ) vNonPreemptiveTick );

	#else

		extern void ( vPreemptiveTick )( void );
		//AT91F_AIC_ConfigureIt( AT91C_ID_SYS, AT91C_AIC_PRIOR_HIGHEST, portINT_LEVEL_SENSITIVE, ( void (*)(void) ) vPreemptiveTick );

	#endif
}
/*-----------------------------------------------------------*/

#if defined( __GNUC__)
#define SWI(x)     { __asm volatile( "SWI %0" : : "I" (x) : "r0", "r1", "r2" ); }

void             xSVC_user_system_call       ( syscall_func_t func, syscall_param_t param)             SWI( SVC_USER_SYSTEM_CALL)
void             xSVC_timer_task_update      ( const tU32 frequency_ratio, const tU32 ratio_scale, const tU32 current_time, const boolean_t update_first) SWI( SVC_TIMER_TASK_UPDATE)
void  gpOS_ISR   portYIELD                   ( void)                                                   SWI( SVC_PORT_YIELD)
#endif

void vSVC_exec_user_system_call( syscall_func_t func, syscall_param_t param)
{
  func( param);
}

void xSVC_handle_user_system_call( svc_calls_t call, unsigned *reg)
{
  if (call == SVC_USER_SYSTEM_CALL)
  {
    vSVC_exec_user_system_call( (syscall_func_t)reg[0], (syscall_param_t)reg[1]);
  }
  if (call == SVC_TIMER_TASK_UPDATE)
  {
    timer_svc_timeout_jobs_update((tS32)reg[0], reg[1], reg[2], reg[3]);
  }
}

/*-----------------------------------------------------------*/

/*
 * Handle the ARM vectors stacks initialisation.
 */

typedef tU64  stack_t;
#define TASK_NAME_SIZE     16

typedef enum
{
  CONFIG_STACK_SVC,
  CONFIG_STACK_UNDEF,
  CONFIG_STACK_ABORT,
  CONFIG_STACK_IRQ,
  CONFIG_STACK_FIQ,
  CONFIG_STACK_NUMBER
} config_stack_t;                  /**< ARM system stacks IDs */

const char __attribute__ ((aligned)) vectors_stack_names[CONFIG_STACK_NUMBER][10] = {
  "OS_SVC_ws",
  "OS_UND_ws",
  "OS_ABT_ws",
  "OS_IRQ_ws",
  "OS_FIQ_ws"
};

#define IRQ_MODE_STACK_SIZE    384  /* size of IRQ mode */
#define SVC_MODE_STACK_SIZE    128  /* size of SVC mode */

extern unsigned int vct_SWIHandlerAddr;
extern unsigned int vct_IRQHandlerAddr;

extern void   svc_SWI_handler ( void);
extern void   interrupt_handler ( void);
extern void   vectors_init_stacks ( void **, unsigned *arm_stacks);

void vOS_vectors_stack_init( gpOS_partition_t *part)
{
  unsigned vectors_stacks[] = {
    SVC_MODE_STACK_SIZE,
    0,
    0,
    IRQ_MODE_STACK_SIZE,
    0
  };

  void *vectors_stack_table[CONFIG_STACK_NUMBER];
  int i;
  int status = TRUE;

  memory_align_p( part, sizeof( stack_t));

  // Reset stacks at some default values
  for( i = 0; (i < CONFIG_STACK_NUMBER) && (status != FALSE); i++)
  {
    if( vectors_stacks[i] != 0)
    {
      vectors_stacks[i] = gpOS_MEMORY_ALIGN( vectors_stacks[i], stack_t);

      memory_align( sizeof( stack_t));
      vectors_stack_table[i] = gpOS_memory_allocate_p( part, vectors_stacks[i]);

      if( vectors_stack_table[i] == NULL)
      {
        status = FALSE;
      }
      else
      {
        _clibs_memset( vectors_stack_table[i], 0x0, TASK_NAME_SIZE);
        _clibs_memset( (char *)vectors_stack_table[i] + TASK_NAME_SIZE, (0x11 * i), vectors_stacks[i] - TASK_NAME_SIZE);
        _clibs_strncpy( vectors_stack_table[i], (void *)&vectors_stack_names[i][0], TASK_NAME_SIZE - 1);
      }
    }
    else
    {
      vectors_stack_table[i] = NULL;
    }
  }

  // Initialize stack pointers of all modes (it exits in USR more)
  if (status == TRUE)
  {
    vectors_init_stacks( vectors_stack_table, vectors_stacks);
  }

  // Install handlers for SWI and IRQ interrupt
  vct_SWIHandlerAddr = (unsigned int)svc_SWI_handler;
  vct_IRQHandlerAddr = (unsigned int)interrupt_handler;
}


/*-----------------------------------------------------------*/

/*
 * Handle profiling stats.
 */
#ifdef FREERTOS_PROFILING

#define gpOS_STATS_INTERVAL      1   /* 1 second interval */

extern unsigned interrupt_occurred_count;
extern unsigned context_switch_count;

void vOS_stats_init( void)
{
  context_switch_count      = 0;
  interrupt_occurred_count  = 0;
  xOSStats.prevPeriodCS      = 0;
  xOSStats.prevPeriodIRQs     = 0;
  xOSStats.curPeriodStartTime   = gpOS_time_now();
  xOSStats.prevPeriodDuration = 1;
  xOSStats.periodExpectedDuration = gpOS_timer_ticks_per_sec() * gpOS_STATS_INTERVAL;
}
#endif

unsigned xOS_stats_get_interrupts_occurred_number()
{
    #ifdef FREERTOS_PROFILING
    return xOSStats.prevPeriodIRQs;
    #else
    return 0;
    #endif
}

unsigned xOS_stats_get_context_switch_number()
{
    #ifdef FREERTOS_PROFILING
    return xOSStats.prevPeriodCS;
    #else
    return 0;
    #endif
}

/*-----------------------------------------------------------*/
#if (configUSE_IDLE_HOOK == 1)
void vApplicationIdleHook( void )
{
  svc_mcu_enter_wfi();
}
#endif

#if (configUSE_TICKLESS_IDLE == 1)
/* Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP(). */
void vApplicationSleep( TickType_t xExpectedIdleTime )
{
    gpOS_clock_t ulCompleteTickPeriods, ulLowPowerTimeBeforeSleep, ulLowPowerTimeAfterSleep, TimeNow = gpOS_time_now();
    eSleepModeStatus eSleepStatus;

    /* Ensure it is still ok to enter the sleep mode. */
    eSleepStatus = eTaskConfirmSleepModeStatus();

    if( eSleepStatus == eAbortSleep )
    {
        /* A task has been moved out of the Blocked state since this macro was
        executed, or a context siwth is being held pending.  Do not enter a
        sleep state.  Restart the tick and exit the critical section. */
        //prvStartTickInterruptTimer();
    }
    else
    {
        /* Remove the tick interrupt and ensure there is no pending interrupt (do it outside critical sections) */
        timer_remove_TickCount();

        if( eSleepStatus == eNoTasksWaitingTimeout )
        {
            svc_mcu_enter_wfi();
        }
        else
        {
            /* The tick flag is set to false before sleeping.  If it is true when sleep
            mode is exited then sleep mode was probably exited because the tick was
            suppressed for the entire xExpectedIdleTime period. */
            ulTickFlag = pdFALSE;

            /* Read the current time from a time source that will remain operational
            while the microcontroller is in a low power state. */
            ulLowPowerTimeBeforeSleep = TimeNow / (portTICK_PERIOD_MS * gpOS_timer_ticks_per_msec());

            timer_generate_TickCount( xExpectedIdleTime, FALSE );

            /* Enter the low power state. */
            svc_mcu_enter_wfi();

            /* Determine how long the microcontroller was actually in a low power
            state for, which will be less than xExpectedIdleTime if the
            microcontroller was brought out of low power mode by an interrupt
            other than that configured by the vSetWakeTimeInterrupt() call.
            Note that the scheduler is suspended before
            portSUPPRESS_TICKS_AND_SLEEP() is called, and resumed when
            portSUPPRESS_TICKS_AND_SLEEP() returns.  Therefore no other tasks will
            execute until this function completes. */
            ulLowPowerTimeAfterSleep = gpOS_time_now() / (portTICK_PERIOD_MS * gpOS_timer_ticks_per_msec());

            if(ulLowPowerTimeAfterSleep >= ulLowPowerTimeBeforeSleep)
            {
              ulCompleteTickPeriods = ulLowPowerTimeAfterSleep - ulLowPowerTimeBeforeSleep;
            }
            else
            {
              ulCompleteTickPeriods = (ulLowPowerTimeAfterSleep + (0xFFFFFFFF/ (portTICK_PERIOD_MS * gpOS_timer_ticks_per_msec()))) - ulLowPowerTimeBeforeSleep;
            }

            if( ulTickFlag != pdFALSE )
            {
                /* The tick interrupt handler will already have pended the tick
                processing in the kernel.  As the pending tick will be processed as
                soon as this function exits, the tick value	maintained by the tick
                is stepped forward by one less than the	time spent sleeping.  The
                actual stepping of the tick appears later in this function. */
                if(ulCompleteTickPeriods > 0)
                {
                    ulCompleteTickPeriods -= 1UL;
                }
            }
            /* Correct the kernels tick count to account for the time the
              microcontroller spent in its low power state. */
            vTaskStepTick( ulCompleteTickPeriods);
            timer_remove_TickCount();

        }

        if(timer_generate_TickCount( 1, FALSE ) == pdFALSE)
        {
            /* it failed because it is too close, so increment the tick and try 2 */
            vTaskStepTick(1);
            timer_generate_TickCount(2, FALSE);
        }
    }
}
#endif

/*-----------------------------------------------------------*/

/*
 * Get OS version.
 */
const char *gpOS_version( void)
{
  return (MCR_VERSION_STRING( "FreeRTOS", tskKERNEL_VERSION_NUMBER));
}

/*}}}  */
