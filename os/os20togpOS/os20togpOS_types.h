/*
 * os20togpOS_types.h
 *
 * Copyright (C) STMicroelectronics Ltd. 2000
 *
 * Message passing.
 */

#ifndef OS20TOGPOS_TYPES_H
#define OS20TOGPOS_TYPES_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define OS20_SUCCESS  gpOS_SUCCESS
#define OS20_FAILURE  gpOS_FAILURE

#define OS20_TRUE     gpOS_TRUE
#define OS20_FALSE    gpOS_FALSE

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

#define os20_stack_t  gpOS_stack_t

#define os20_error_t  gpOS_error_t
#define os20_bool_t   gpOS_bool_t

//typedef unsigned int gpOS_cpsr_t;     /**< OS20 current program status register type */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* OS20TOGPOS_TYPES_H */
