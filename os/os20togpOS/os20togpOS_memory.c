/**
 * @file    os20togpOS_memory.c
 * @brief   OS20 memory to genericOS memory wrapper.
 *
 * @addtogroup OS20 to genericOS wrapper
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
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

void *memory_allocate( const tSize size)
{
	return gpOS_memory_allocate(size);
}

void memory_deallocate( void *mem_ptr)
{
	gpOS_memory_deallocate(mem_ptr);
}

tSize memory_getheapsize( void)
{
	return gpOS_memory_getheapsize();
}

tSize memory_getheapfree( void)
{
	return gpOS_memory_getheapfree();
}

tSize memory_getheaprequested( void)
{
	return gpOS_memory_getheaprequested();
}

void *memory_allocate_p( partition_t *partition_ptr, const tSize size)
{
	return gpOS_memory_allocate_p(partition_ptr, size);
}

void memory_deallocate_p( partition_t *partition_ptr, void *mem_ptr)
{
	gpOS_memory_deallocate_p(partition_ptr, mem_ptr);
}

tSize memory_getheapsize_p( partition_t *partition_ptr)
{
	return gpOS_memory_getheapsize_p(partition_ptr);
}

tSize memory_getheapfree_p( partition_t *partition_ptr)
{
	return gpOS_memory_getheapfree_p(partition_ptr);
}

tSize memory_getheaprequested_p( partition_t *partition_ptr)
{
	return gpOS_memory_getheaprequested_p(partition_ptr);
}

partition_t *memory_create_partition( os20_memory_type_t type, void *part_start_addr, const tSize part_size)
{
	return gpOS_memory_create_partition(type, part_start_addr, part_size);
}

partition_t *memory_getnextpartition( partition_t *partition_ptr)
{
	return gpOS_memory_getnextpartition(partition_ptr);
}





