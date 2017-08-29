/**
 * @file    interrupt.h
 * @brief   OS20 interrupt handling definitions and macros.
 *
 * @addtogroup OS20
 */

#ifndef gpOS_INTERRUPT_H
#define gpOS_INTERRUPT_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS_memory.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define gpOS_INTERRUPT_NOPRIORITY       ((tUInt)0xffff)

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef tUInt gpOS_interrupt_line_t;                    /**< Interrupt line */
typedef tUInt gpOS_interrupt_priority_t;                /**< Interrupt priority */

typedef void  (*gpOS_interrupt_callback_t)  (void *);   /**< Interrupt callback function */
typedef void  (*gpOS_interrupt_hook_t)      (void);     /**< Interrupt enter/leave hook function */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern void gpOS_interrupt_init      ( gpOS_partition_t *part, const tSize stack_size);
extern void gpOS_interrupt_set_hooks ( gpOS_interrupt_hook_t, gpOS_interrupt_hook_t);
extern void gpOS_interrupt_install   ( gpOS_interrupt_line_t line, gpOS_interrupt_priority_t priority, gpOS_interrupt_callback_t callback_func, void *callback_param);
extern void gpOS_interrupt_uninstall ( gpOS_interrupt_line_t line);
extern void gpOS_interrupt_enable    ( gpOS_interrupt_line_t line);
extern void gpOS_interrupt_disable   ( gpOS_interrupt_line_t line);

extern void gpOS_interrupt_lock      ( void);
extern void gpOS_interrupt_unlock    ( void);

#endif /* gpOS_INTERRUPT_H */
