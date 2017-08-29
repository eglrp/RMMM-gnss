/**
 * @file    freeRTOS_interrupt.c
 * @brief   FreeRTOS interrupt handling implementation.
 *
 */

/*****************************************************************************
   includes
*****************************************************************************/

#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "FRi.h"

#include "gpOS_bsp.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

#if defined( FREERTOS_IRQ_HOOKS)
extern gpOS_interrupt_hook_t interrupt_enter_hook;    /**< Interrupt enter hook pointer */
extern gpOS_interrupt_hook_t interrupt_leave_hook;    /**< Interrupt leave hook pointer */
#endif

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/*{{{  gpOS_interrupt_install()*/
/********************************************//**
 * \brief   Install an interrupt handler on given
 *          line with given priority
 *
 * \param   line      Interrupt line
 * \param   priority  Interrupt priority
 * \param   func_ptr  Interrupt callback function
 * \param   arg_ptr   Interrupt callback parameter
 * \return  void
 *
 ***********************************************/
void gpOS_interrupt_install( gpOS_interrupt_line_t line, gpOS_interrupt_priority_t priority, gpOS_interrupt_callback_t func_ptr, void *arg_ptr)
{
  gpOS_bsp_interrupt_install( line, priority, func_ptr, arg_ptr);
}
/*}}}  */
/*{{{  gpOS_interrupt_uninstall()*/
/********************************************//**
 * \brief   Removes an handler on given interrupt line
 *
 * \param   line  Interrupt line to clear
 * \return  void
 *
 ***********************************************/
void gpOS_interrupt_uninstall( gpOS_interrupt_line_t line)
{
  gpOS_bsp_interrupt_uninstall( line);
}

/********************************************//**
 * \brief   Enables interrupt line
 *
 * \param   line  Interrupt line to clear
 * \return  void
 *
 ***********************************************/
void gpOS_interrupt_enable( gpOS_interrupt_line_t line)
{
  gpOS_bsp_interrupt_enable( line);
}

/********************************************//**
 * \brief   Disables interrupt line
 *
 * \param   line  Interrupt line to clear
 * \return  void
 *
 ***********************************************/
void gpOS_interrupt_disable( gpOS_interrupt_line_t line)
{
  gpOS_bsp_interrupt_disable( line);
}

/********************************************//**
 * \brief   Set hooks for enter and leave sections in interrupt handler
 *
 * \param   enter_hook  Hook to be called when entering interrupt
 * \param   leave_hook  Hook to be called when leaving interrupt
 * \return  void
 *
 ***********************************************/
void gpOS_interrupt_set_hooks( gpOS_interrupt_hook_t enter_hook, gpOS_interrupt_hook_t leave_hook)
{
  #if defined( FREERTOS_IRQ_HOOKS )
  interrupt_enter_hook = enter_hook;
  interrupt_leave_hook = leave_hook;
  #endif
}

#if defined( FREERTOS_IRQ_HOOKS )
/********************************************//**
 * \brief   Set empty enter hooks
 *
 ***********************************************/
gpOS_ISR void IRQ_enter_hook_empty( void )
{
#if defined( __GNUC__)
  portNOP();
#else
#pragma arm
  portNOP();
#endif
}

/********************************************//**
 * \brief   Set empty exit hooks
 *
 ***********************************************/
gpOS_ISR void IRQ_leave_hook_empty( void )
{
#if defined( __GNUC__)
  portNOP();
#else
#pragma arm
  portNOP();
#endif
}
#endif

gpOS_ISR void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	/* This will get called if a stack overflow is detected during the context
	switch.  Set configCHECK_FOR_STACK_OVERFLOWS to 2 to also check for stack
	problems within nested interrupts, but only do this for debug purposes as
	it will increase the context switch time. */

	( void ) pxTask;
	( void ) pcTaskName;

  vFreeRTOS_Abort();
}

/*}}}  */
