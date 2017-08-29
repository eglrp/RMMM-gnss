/**
 * @file    os20togpOS_memoryi.h
 * @brief   OS20 memory internal definitions and macros.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_MEMORYI_H
#define OS20TOGPOS_MEMORYI_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define OS20_MEMORY_ALIGN             gpOS_MEMORY_ALIGN

#define OS20_MEMORY_TYPE_SYSTEM_HEAP  gpOS_MEMORY_TYPE_SYSTEM_HEAP
#define OS20_MEMORY_TYPE_SIMPLE       gpOS_MEMORY_TYPE_SIMPLE
#define OS20_MEMORY_TYPE_HEAP         gpOS_MEMORY_TYPE_HEAP

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

#define os20_memory_type_t    gpOS_memory_type_t
#define partition_t           gpOS_partition_t

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* OS20TOGPOS_MEMORYI_H */

