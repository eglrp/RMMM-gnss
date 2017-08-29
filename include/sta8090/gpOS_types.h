/*
 * os20types.h
 *
 * Copyright (C) STMicroelectronics Ltd. 2000
 *
 * Message passing.
 */

#ifndef gpOS_TYPES_H
#define gpOS_TYPES_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "macros.h"
#include "typedefs.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/
/*lint -esym(9071,NULL) : defined macro 'NULL' is reserved to the compiler [MISRA 2012 Rule 21.1, required]*/
#ifndef   NULL
#define   NULL                  ((void*)0)
#endif

#define gpOS_MODE_USR           0x10
#define gpOS_MODE_FIQ           0x11
#define gpOS_MODE_IRQ           0x12
#define gpOS_MODE_SVC           0x13
#define gpOS_MODE_ABT           0x17
#define gpOS_MODE_UND           0x1B
#define gpOS_MODE_SYS           0x1F // available on ARM Arch 4 and later

#define gpOS_MODE_MASK          0x1f // bit mask for mode

#define gpOS_I_BIT              0x80 // when I bit is set, IRQ is disabled
#define gpOS_F_BIT              0x40 // when F bit is set, FIQ is disabled
#define gpOS_T_BIT              0x20

#define BYTES_PER_WORD          4
#define WORD_MASK               (-BYTES_PER_WORD)

#if (__ARMCC_VERSION >= 300586) || defined( __GNUC__)
#define gpOS_ISR                __attribute__((section ("OS_ISR_REGION")))
#define gpOS_FAST               __attribute__((section ("OS_FAST_REGION")))
#define gpOS_WEAK               __attribute__((weak))
#else
#define gpOS_ISR
#define gpOS_FAST
#define gpOS_WEAK
#endif

#define gpOS_SUCCESS            0
#define gpOS_FAILURE            -1

#define gpOS_TRUE               (1 == 1)
#define gpOS_FALSE              (0 == 1)

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef tInt  gpOS_error_t;         /**< OS error type */

typedef tInt  gpOS_bool_t;          /**< OS boolean type */

typedef tU64  gpOS_stack_t;         /**<  */

typedef tUInt gpOS_cpsr_t;          /**< OS current program status register type */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* gpOS_TYPES_H */
