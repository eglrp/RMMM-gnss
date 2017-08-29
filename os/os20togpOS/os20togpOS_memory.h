/**
 * @file    os20togpOS_memory.h
 * @brief   OS20 memory functions definitions.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_MEMORY_H
#define OS20TOGPOS_MEMORY_H

/*****************************************************************************
   includes
*****************************************************************************/
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

#define memory_allocate(size)                      gpOS_memory_allocate(size)
#define memory_deallocate(mem_ptr)                 gpOS_memory_deallocate(mem_ptr)
#define memory_getheapsize()                       gpOS_memory_getheapsize()
#define memory_getheapfree()                       gpOS_memory_getheapfree()
#define memory_getheaprequested()                  gpOS_memory_getheaprequested()

#define memory_allocate_p(part, size)              gpOS_memory_allocate_p(part, size)
#define memory_deallocate_p(part, mem_ptr)         gpOS_memory_deallocate_p(part, mem_ptr)
#define memory_getheapsize_p(part)                 gpOS_memory_getheapsize_p(part)
#define memory_getheapfree_p(part)                 gpOS_memory_getheapfree_p(part)
#define memory_getheaprequested_p(partition_ptr)   gpOS_memory_getheaprequested_p(partition_ptr)

#define memory_create_partition(type, part_start_addr, part_size)  gpOS_memory_create_partition(type, part_start_addr, part_size)
#define memory_getnextpartition(part)              gpOS_memory_getnextpartition(part)

#define memory_partition_extend_heap(min_size, base_addr) gpOS_memory_partition_extend_heap  (min_size, base_addr)
#define memory_lock()                              gpOS_memory_lock()
#define memory_unlock()                            gpOS_memory_unlock()

#endif /* OS20TOGPOS_MEMORY_H */

