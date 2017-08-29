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

/*
	Changes from V3.2.3

	+ Modified portENTER_SWITCHING_ISR() to allow use with GCC V4.0.1.

	Changes from V3.2.4

	+ Removed the use of the %0 parameter within the assembler macros and
	  replaced them with hard coded registers.  This will ensure the
	  assembler does not select the link register as the temp register as
	  was occasionally happening previously.

	+ The assembler statements are now included in a single asm block rather
	  than each line having its own asm block.

	Changes from V4.5.0

	+ Removed the portENTER_SWITCHING_ISR() and portEXIT_SWITCHING_ISR() macros
	  and replaced them with portYIELD_FROM_ISR() macro.  Application code
	  should now make use of the portSAVE_CONTEXT() and portRESTORE_CONTEXT()
	  macros as per the V4.5.1 demo code.
*/

#ifndef PORTMACRO_H
#define PORTMACRO_H

#include "lld.h"

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	uint32_t
#define portBASE_TYPE	long

typedef portSTACK_TYPE StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
typedef struct osTimeStats_s {
  unsigned long periodExpectedDuration;
  unsigned long prevPeriodDuration;
  unsigned long curPeriodStartTime;
  unsigned long prevPeriodCS;                    /**< last number of context switches */
  unsigned long prevPeriodIRQs;                  /**< number of IRQs handled */
  volatile unsigned long prevPeriodIRQDuration;
} osTimeStats_t;
#endif

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
extern osTimeStats_t xOSStats;
//extern void * volatile pxCurrentTCB = NULL;
#endif

typedef struct taskTagCB_s {
  void *Tag2nd;
  struct taskTagCB_s *next;
  char *tName;
#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
  unsigned long prevPeriodSum;   // Cummulated RunTime during Previous Period
  unsigned long timeStart;   // The last time task was switched in minus currentPeriod runtime
  unsigned long timeEnd;     // The last time task was switched out
#endif
} taskTagCB_t;

extern taskTagCB_t *tagList;

#if( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif
/*-----------------------------------------------------------*/

/* Hardware specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT			8
#if defined( __GNUC__)
#define portNOP()					asm volatile ( "NOP" )
#else
#define portNOP()	 				__asm ( "NOP" )
#endif


/*
 * Handle the freeRTOS scheduling with portYIELD and the function calls in supervisor mode.
 */

/*{{{  typedef*/
typedef enum svc_calls_e
{
  SVC_INTERRUPT_DISABLE,
  SVC_INTERRUPT_ENABLE,
  SVC_USER_SYSTEM_CALL,
  SVC_TIMER_TASK_UPDATE,
  SVC_PORT_YIELD            /* To be kept aligned with PORT_YIELD_SWI definiton in FR_defines.asm */
}
svc_calls_t;
/*}}}  */

typedef void *syscall_param_t;                      /**< System call parameter type */
typedef void (*syscall_func_t)( syscall_param_t);   /**< System call type */

#if defined( __ARMCC_VERSION)
#define __SWI_TYPE(x) __swi (x)
#endif

#if defined( __GNUC__)
#define __SWI_TYPE(x) extern
#endif

__SWI_TYPE(SVC_PORT_YIELD)         void  portYIELD ( void);
__SWI_TYPE(SVC_USER_SYSTEM_CALL)   void  xSVC_user_system_call ( syscall_func_t func, syscall_param_t param);
__SWI_TYPE(SVC_TIMER_TASK_UPDATE)  void  xSVC_timer_task_update      ( const tU32 frequency_ratio, const tU32 ratio_scale, const tU32 current_time, const boolean_t update_first);

extern void     vApplicationSleep (TickType_t);

extern volatile uint32_t ulCriticalNesting;
extern volatile uint32_t itTimePeriodCumul;
extern volatile uint32_t itTimeCumul;
extern volatile uint32_t itTimeStart;

/*
 * These define the timer to use for generating the tick interrupt.
 * They are put in this file so they can be shared between "port.c"
 * and "portisr.c".
 */
#define portTIMER_REG_BASE_PTR		AT91C_BASE_TC0
#define portTIMER_CLK_ENABLE_BIT	AT91C_PS_TC0
#define portTIMER_AIC_CHANNEL		( ( uint32_t ) 4 )

/*-----------------------------------------------------------*/

/* Task utilities. */

/*
 * portRESTORE_CONTEXT, portRESTORE_CONTEXT, portENTER_SWITCHING_ISR
 * and portEXIT_SWITCHING_ISR can only be called from ARM mode, but
 * are included here for efficiency.  An attempt to call one from
 * THUMB mode code will result in a compile time error.
 */

#if defined( __GNUC__)
#define portRESTORE_CONTEXT()											\
{																		\
extern volatile void * volatile pxCurrentTCB;							\
extern volatile uint32_t ulCriticalNesting;					\
																		\
	/* Set the LR to the task stack. */									\
	asm volatile (														\
	"LDR		R0, =pxCurrentTCB								\n\t"	\
	"LDR		R0, [R0]										\n\t"	\
	"LDR		LR, [R0]										\n\t"	\
																		\
	/* The critical nesting depth is the first item on the stack. */	\
	/* Load it into the ulCriticalNesting variable. */					\
	"LDR		R0, =ulCriticalNesting							\n\t"	\
	"LDMFD	LR!, {R1}											\n\t"	\
	"STR		R1, [R0]										\n\t"	\
																		\
	/* Get the SPSR from the stack. */									\
	"LDMFD	LR!, {R0}											\n\t"	\
	"MSR		SPSR, R0										\n\t"	\
																		\
	/* Restore all system mode registers for the task. */				\
	"LDMFD	LR, {R0-R14}^										\n\t"	\
	"NOP														\n\t"	\
																		\
	/* Restore the return address. */									\
	"LDR		LR, [LR, #+60]									\n\t"	\
																		\
	/* And return - correcting the offset in the LR to obtain the */	\
	/* correct address. */												\
	"SUBS	PC, LR, #4											\n\t"	\
	);																	\
	( void ) ulCriticalNesting;											\
	( void ) pxCurrentTCB;												\
}
/*-----------------------------------------------------------*/

#define portSAVE_CONTEXT()												\
{																		\
extern volatile void * volatile pxCurrentTCB;							\
extern volatile uint32_t ulCriticalNesting;					\
																		\
	/* Push R0 as we are going to use the register. */					\
	asm volatile (														\
	"STMDB	SP!, {R0}											\n\t"	\
																		\
	/* Set R0 to point to the task stack pointer. */					\
	"STMDB	SP,{SP}^											\n\t"	\
	"NOP														\n\t"	\
	"SUB	SP, SP, #4											\n\t"	\
	"LDMIA	SP!,{R0}											\n\t"	\
																		\
	/* Push the return address onto the stack. */						\
	"STMDB	R0!, {LR}											\n\t"	\
																		\
	/* Now we have saved LR we can use it instead of R0. */				\
	"MOV	LR, R0												\n\t"	\
																		\
	/* Pop R0 so we can save it onto the system mode stack. */			\
	"LDMIA	SP!, {R0}											\n\t"	\
																		\
	/* Push all the system mode registers onto the task stack. */		\
	"STMDB	LR,{R0-LR}^											\n\t"	\
	"NOP														\n\t"	\
	"SUB	LR, LR, #60											\n\t"	\
																		\
	/* Push the SPSR onto the task stack. */							\
	"MRS	R0, SPSR											\n\t"	\
	"STMDB	LR!, {R0}											\n\t"	\
																		\
	"LDR	R0, =ulCriticalNesting								\n\t"	\
	"LDR	R0, [R0]											\n\t"	\
	"STMDB	LR!, {R0}											\n\t"	\
																		\
	/* Store the new top of stack for the task. */						\
	"LDR	R0, =pxCurrentTCB									\n\t"	\
	"LDR	R0, [R0]											\n\t"	\
	"STR	LR, [R0]											\n\t"	\
	);																	\
	( void ) ulCriticalNesting;											\
	( void ) pxCurrentTCB;												\
}

#define portYIELD_FROM_ISR() vTaskSwitchContext()

/* Critical section handling. */

/*
 * The interrupt management utilities can only be called from ARM mode.  When
 * THUMB_INTERWORK is defined the utilities are defined as functions in
 * portISR.c to ensure a switch to ARM mode.  When THUMB_INTERWORK is not
 * defined then the utilities are defined as macros here - as per other ports.
 */

#ifdef THUMB_INTERWORK

	extern void vPortDisableInterruptsFromThumb( void ) __attribute__ ((naked));
	extern void vPortEnableInterruptsFromThumb( void ) __attribute__ ((naked));

	#define portDISABLE_INTERRUPTS()	vPortDisableInterruptsFromThumb()
	#define portENABLE_INTERRUPTS()		vPortEnableInterruptsFromThumb()

#else

	#define portDISABLE_INTERRUPTS()											\
		asm volatile (															\
			"STMDB	SP!, {R0}		\n\t"	/* Push R0.						*/	\
			"MRS	R0, CPSR		\n\t"	/* Get CPSR.					*/	\
			"ORR	R0, R0, #0xC0	\n\t"	/* Disable IRQ, FIQ.			*/	\
			"MSR	CPSR, R0		\n\t"	/* Write back modified value.	*/	\
			"LDMIA	SP!, {R0}			" )	/* Pop R0.						*/

	#define portENABLE_INTERRUPTS()												\
		asm volatile (															\
			"STMDB	SP!, {R0}		\n\t"	/* Push R0.						*/	\
			"MRS	R0, CPSR		\n\t"	/* Get CPSR.					*/	\
			"BIC	R0, R0, #0xC0	\n\t"	/* Enable IRQ, FIQ.				*/	\
			"MSR	CPSR, R0		\n\t"	/* Write back modified value.	*/	\
			"LDMIA	SP!, {R0}			" )	/* Pop R0.						*/

#endif /* THUMB_INTERWORK */

#else /* __GNUC__ */

extern void vPortDisableInterruptsArm( void );
extern void vPortEnableInterruptsArm( void );

	#define portDISABLE_INTERRUPTS()	vPortDisableInterruptsArm();
	#define portENABLE_INTERRUPTS()		vPortEnableInterruptsArm();

#endif /* __GNUC__ */

extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );

#define portENTER_CRITICAL()		vPortEnterCritical();
#define portEXIT_CRITICAL()			vPortExitCritical();

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

/*-----------------------------------------------------------*/

/* Time stat utilities */

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
#define TIME_STAT_INIT_TAG_CUSTOM(tagCB) \
  tagCB->prevPeriodSum = 0; \
  tagCB->timeStart = 0; \
  tagCB->timeEnd = 0;
#else
#define TIME_STAT_INIT_TAG_CUSTOM(tagTCB)
#endif

#define traceTASK_CREATE(thisTCB)                                \
  {                                                              \
    taskTagCB_t *tagCB = pvPortMalloc( sizeof( taskTagCB_t ) );  \
    tagCB->Tag2nd = thisTCB->pxTaskTag;                          \
    tagCB->tName = thisTCB->pcTaskName;                          \
    TIME_STAT_INIT_TAG_CUSTOM(tagCB)                             \
    tagCB->next = tagList;                                       \
    tagList = tagCB;                                             \
    thisTCB->pxTaskTag = ( TaskHookFunction_t ) tagCB;           \
  }

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )

#define traceTASK_SWITCHED_OUT()                                 \
      {                                                          \
        extern taskTagCB_t *prevTaskTag;                         \
        prevTaskTag = (taskTagCB_t *)pxCurrentTCB->pxTaskTag;    \
      }

#endif

#define traceTASK_SWITCHED_IN()                                                   \
      {                                                                           \
        extern taskTagCB_t *prevTaskTag;                                          \
        extern unsigned int gpOS_time_now( void);                                  \
        extern volatile uint32_t itTimeCumul;                                     \
        extern unsigned context_switch_count;                                     \
        taskTagCB_t *tagCB;                                                       \
        int32_t CurPeriodDuration;                                                \
        uint32_t tNow = gpOS_time_now();                                            \
        /* Mark previous task*/                                                   \
        if(NULL != prevTaskTag) {                                                 \
          prevTaskTag->timeEnd = tNow - itTimeCumul;                              \
        }                                                                         \
        itTimeCumul = 0;                                                          \
        context_switch_count++;                                                   \
        CurPeriodDuration = (int32_t)(tNow - xOSStats.curPeriodStartTime);        \
        /* Is it time to collect data ? */                                        \
        if(CurPeriodDuration >= (int32_t)xOSStats.periodExpectedDuration) {       \
          extern unsigned interrupt_occurred_count;                               \
          extern volatile uint32_t itTimePeriodCumul;                             \
          xOSStats.prevPeriodDuration = CurPeriodDuration;                        \
          xOSStats.prevPeriodIRQDuration = itTimePeriodCumul;                     \
          xOSStats.curPeriodStartTime = tNow;                                     \
          xOSStats.prevPeriodCS = context_switch_count;                           \
          xOSStats.prevPeriodIRQs = interrupt_occurred_count;                     \
          context_switch_count      = 0;                                          \
          interrupt_occurred_count  = 0;                                          \
          itTimePeriodCumul         = 0;                                          \
          for(tagCB = tagList; tagCB != NULL; tagCB = tagCB->next) {              \
            tagCB->prevPeriodSum = tagCB->timeEnd - tagCB->timeStart;             \
            tagCB->timeStart = tagCB->timeEnd;                                    \
          }                                                                       \
        }                                                                         \
        /* Prepare new task*/                                                     \
        tagCB = (taskTagCB_t *)pxCurrentTCB->pxTaskTag;                           \
        tagCB->timeStart = tNow - (tagCB->timeEnd - tagCB->timeStart);            \
      }

/*-----------------------------------------------------------*/

/* Tickless utilities */

#if (configUSE_TICKLESS_IDLE == 1)
#define portSUPPRESS_TICKS_AND_SLEEP( xIdleTime ) vApplicationSleep( xIdleTime )
#endif

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */

