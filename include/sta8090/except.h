/**
 * @file    except.h
 * @brief   OS20 exception handling definitions and macros.
 *
 * @addtogroup OS20
 */

#ifndef OS20_EXCEPT_H
#define OS20_EXCEPT_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum os20_exceptions_e {
  OS20_EXCEPTION_RESET,
  OS20_EXCEPTION_UND,
  OS20_EXCEPTION_SVC,
  OS20_EXCEPTION_PFABT,
  OS20_EXCEPTION_DABT,
  OS20_EXCEPTION_IRQ,
  OS20_EXCEPTION_FIQ
} os20_exceptions_t;                                          /**< Exception identifier */

typedef void (*os20_exception_vector_t)(void);                /**< Exception vector type */
typedef void *gpOS_syscall_param_t;                           /**< System call parameter type */
typedef void (*gpOS_syscall_func_t)( gpOS_syscall_param_t);   /**< System call type */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern void   gpOS_exception_init    ( os20_exceptions_t arm_exception, os20_exception_vector_t vector);
extern void   gpOS_exception_clear   ( os20_exceptions_t arm_exception);

#endif /* OS20_EXCEPT_H */
