/**
 * @file    os20togpOS_interrupt.h
 * @brief   OS20 interrupt functions definitions.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_INTERRUPT_H
#define OS20TOGPOS_INTERRUPT_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "os20togpOS_interrupti.h"

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

#define interrupt_init(part, stack_size)                                  gpOS_interrupt_init(part, stack_size)
#define interrupt_install(line, priority, callback_func, callback_param)  gpOS_interrupt_install(line, priority, callback_func, callback_param)
#define interrupt_uninstall(line)                                         gpOS_interrupt_uninstall(line)
#define interrupt_enable(line)                                            gpOS_interrupt_enable(line)
#define interrupt_disable(line)                                           gpOS_interrupt_disable(line)

#if defined( FREERTOS_IRQ_HOOKS )
#define interrupt_set_hooks(enter_hook, leave_hook)                       gpOS_interrupt_set_hooks(enter_hook, leave_hook)
#endif

#define interrupt_lock()                                                  gpOS_interrupt_lock()
#define interrupt_unlock()                                                gpOS_interrupt_unlock()

#endif /* OS20TOGPOS_INTERRUPT_H */
