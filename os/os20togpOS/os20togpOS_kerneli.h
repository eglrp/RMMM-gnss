/**
 * @file    os20togpOS_kerneli.h
 * @brief   OS20 kernel internal definitions and macros.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_KERNELI_H
#define OS20TOGPOS_KERNELI_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define OS20_EXCEPTION_RESET      gpOS_EXCEPTION_RESET
#define OS20_EXCEPTION_UND        gpOS_EXCEPTION_UND
#define OS20_EXCEPTION_SVC        gpOS_EXCEPTION_SVC
#define OS20_EXCEPTION_PFABT      gpOS_EXCEPTION_PFABT
#define OS20_EXCEPTION_DABT       gpOS_EXCEPTION_DABT
#define OS20_EXCEPTION_IRQ        gpOS_EXCEPTION_IRQ
#define OS20_EXCEPTION_FIQ        gpOS_EXCEPTION_FIQ

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

#define os20_exceptions_t         gpOS_exceptions_t

#define os20_exception_vector_t   gpOS_exception_vector_t
#define os20_syscall_param_t      gpOS_syscall_param_t
#define os20_syscall_func_t       gpOS_syscall_func_t

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* OS20TOGPOS_KERNELI_H */
