/**
 * @file    gpOS_memory.h
 * @brief   OS20 memory handling definitions and macros.
 *
 * @addtogroup OS20
 */

#ifndef gpOS_MEMORY_H
#define gpOS_MEMORY_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS_types.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define gpOS_MEMORY_ALIGN( size, type)   (( (size) + ( sizeof(type) - 1)) & ~( sizeof(type) - 1))     /**< macro to align wanted "size" at needed "type" alignment */

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef struct gpOS_partition_s gpOS_partition_t;       /**< OS memory partition type */

#if defined( OS_LITE )
typedef tUInt gpOS_memory_type_t;
#else
typedef enum gpOS_memory_type_e
{
  gpOS_MEMORY_TYPE_SYSTEM_HEAP,
  gpOS_MEMORY_TYPE_SIMPLE,
  gpOS_MEMORY_TYPE_HEAP
} gpOS_memory_type_t;
#endif

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern void *         gpOS_memory_allocate               ( const tSize size);
extern void           gpOS_memory_deallocate             ( void *mem_ptr);
extern tSize          gpOS_memory_getheapsize            ( void);
extern tSize          gpOS_memory_getheapfree            ( void);
extern tSize          gpOS_memory_getheaprequested       ( void);

extern void *         gpOS_memory_allocate_p             ( gpOS_partition_t *part, const tSize size);
extern void           gpOS_memory_deallocate_p           ( gpOS_partition_t *part, void *mem_ptr);
extern tSize          gpOS_memory_getheapsize_p          ( gpOS_partition_t *part);
extern tSize          gpOS_memory_getheapfree_p          ( gpOS_partition_t *part);
extern tSize          gpOS_memory_getheaprequested_p     ( gpOS_partition_t *part);

extern void           gpOS_memory_lock                   ( void);
extern void           gpOS_memory_unlock                 ( void);

extern gpOS_partition_t * gpOS_memory_create_partition      ( gpOS_memory_type_t type, void *part_start_addr, const tSize part_size);
extern gpOS_partition_t * gpOS_memory_getnextpartition      ( gpOS_partition_t *part);
extern tUInt              gpOS_memory_partition_extend_heap ( tUInt min_size, void **base_addr);

#endif /* gpOS_MEMORY_H */

