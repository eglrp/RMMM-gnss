/**
 * @file    os20togpOS_interrupt.c
 * @brief   OS20 interrupt to genericOS interrupt wrapper.
 *
 * @addtogroup OS20 to genericOS wrapper
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "os20togpOS_interrupti.h"
#include "os20togpOS_memoryi.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

void interrupt_init( partition_t *part, const tSize stack_size)
{
	gpOS_interrupt_init(part, stack_size);
}

void interrupt_install( os20_interrupt_line_t line, os20_interrupt_priority_t priority, os20_interrupt_callback_t func_ptr, void *arg_ptr)
{
	gpOS_interrupt_install(line, priority, func_ptr, arg_ptr);
}

void interrupt_uninstall( os20_interrupt_line_t line)
{
	gpOS_interrupt_uninstall(line);
}

void interrupt_enable( os20_interrupt_line_t line)
{
	gpOS_interrupt_enable(line);
}

void interrupt_disable( os20_interrupt_line_t line)
{
	gpOS_interrupt_disable(line);
}

#if defined( FREERTOS_IRQ_HOOKS )
extern void interrupt_set_hooks ( os20_interrupt_hook_t, os20_interrupt_hook_t)
{
	gpOS_interrupt_set_hooks(enter_hook, leave_hook);
}
#endif

void interrupt_lock( void)
{
	gpOS_interrupt_lock();
}

void interrupt_unlock( void)
{
	gpOS_interrupt_unlock();
}
