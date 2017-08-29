/**
 * @file    freeRTOS_memory.c
 * @brief   freeRTOS memory handling implementation.
 *
 */

/*****************************************************************************
   includes
*****************************************************************************/

#include "clibs.h"

#include "typedefs.h"
#include "macros.h"

#include "FreeRTOS.h"
#include "FR_memory.h"
#include "FR_memoryi.h"
#include "semphr.h"
#include "task.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

#if defined( __ARMCC_VERSION)
extern tUInt Image$$OS_HEAP_AREA$$Base;
extern tUInt Image$$OS_STACK_AREA$$Base;

#define OS_HEAP_START   ((tUInt)(&Image$$OS_HEAP_AREA$$Base))
#define OS_HEAP_END     ((tUInt)(&Image$$OS_STACK_AREA$$Base))
#endif

#if defined( __GNUC__)
extern tUInt __os_heap_area_start__;
extern tUInt __os_stack_area_limit__;

#define OS_HEAP_START   ((tUInt)(&__os_heap_area_start__))
#define OS_HEAP_END     ((tUInt)(&__os_stack_area_limit__))
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static gpOS_partition_t os_root_partition;
static SemaphoreHandle_t xmemory_mutex = NULL;
static gpOS_partition_t *optUsedPartition = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/
gpOS_partition_t *memgt_mallopt_set(gpOS_partition_t *newPart)
{
  gpOS_partition_t *oldPartition = optUsedPartition;

  // Prevent Other task to use memory API
  vTaskSuspendAll();

  optUsedPartition = newPart;

  return oldPartition;
}

/*   */
gpOS_partition_t *memgt_mallopt_restore(gpOS_partition_t *restorePart)
{
  gpOS_partition_t *oldPartition = optUsedPartition;

  optUsedPartition = restorePart;

  // Allow Memory API to other tasks
  xTaskResumeAll();

  return oldPartition;
}


/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

void vPortFree( void *mem_ptr )
{
  gpOS_memory_deallocate_p( optUsedPartition, mem_ptr);
}

void *pvPortMalloc( size_t xWantedSize )
{
  void *data = gpOS_memory_allocate_p( optUsedPartition, xWantedSize);

  return data;
}

void gpOS_memory_init( void)
{
  os_root_partition.start_ptr   = (void *)OS_HEAP_START;
  os_root_partition.total_size  = (unsigned int)(OS_HEAP_END - OS_HEAP_START);

  os_root_partition.type      = gpOS_MEMORY_TYPE_HEAP;
  os_root_partition.next      = NULL;
  os_root_partition.next_same = NULL;

  os_root_partition.used      = FALSE;
  os_root_partition.req_size  = 0;

  os_root_partition.curr_ptr  = os_root_partition.start_ptr;
  os_root_partition.curr_size = os_root_partition.total_size;

  memory_align_p( &os_root_partition, sizeof( unsigned));
}

void memory_align_p( gpOS_partition_t *partition_ptr, const unsigned int alignment)
{
  gpOS_partition_t *memory_partition_ptr = (gpOS_partition_t *)partition_ptr;

  if( alignment < sizeof( unsigned))
  {
    return;
  }

  if( memory_partition_ptr == NULL)
  {
    memory_partition_ptr = &os_root_partition;
  }

  gpOS_memory_lock();

  if( memory_partition_ptr->type == gpOS_MEMORY_TYPE_SIMPLE)
  {
    if( alignment < memory_partition_ptr->curr_size)
    {
      unsigned int aligned_ptr;
      unsigned int aligned_size;

      aligned_ptr = ((unsigned int)memory_partition_ptr->curr_ptr + (alignment - 1)) & ~(alignment - 1);
      aligned_size = aligned_ptr - (unsigned int)memory_partition_ptr->curr_ptr;

      memory_partition_ptr->curr_size -= aligned_size;
      memory_partition_ptr->req_size += aligned_size;

      memory_partition_ptr->curr_ptr = (void *)aligned_ptr;
    }
  }
  else if( memory_partition_ptr->type == gpOS_MEMORY_TYPE_HEAP)
  {
    // No effect, alignment is handled by runtime library
  }

  gpOS_memory_unlock();
}

unsigned gpOS_memory_partition_extend_heap( unsigned min_size, void **base_addr)
{
  // root partition is by default of heap type
  gpOS_partition_t *memory_partition_ptr = &os_root_partition;
  unsigned size = 0;
  boolean_t space_found = FALSE;

  *base_addr = NULL;

  // look for available space
  while( (space_found == FALSE) && (memory_partition_ptr != NULL))
  {
    if( (memory_partition_ptr->used == TRUE) || (min_size > memory_partition_ptr->curr_size))
    {
      // avoid this partition to be used anymore
      memory_partition_ptr->used = TRUE;
      memory_partition_ptr = memory_partition_ptr->next_same;
    }
    else
    {
      space_found = TRUE;
    }
  }

  if( space_found == TRUE)
  {
    gpOS_memory_lock();

    *base_addr  = memory_partition_ptr->curr_ptr;
    size        = min_size;

    memory_partition_ptr->curr_size -= size;
    memory_partition_ptr->curr_ptr = (void *)((unsigned int)memory_partition_ptr->curr_ptr + size);

    gpOS_memory_unlock();
  }

  return size;
}

void gpOS_memory_protection_init( void)
{
  xmemory_mutex = xSemaphoreCreateRecursiveMutex();
}

void *gpOS_memory_allocate( const size_t size)
{
  return gpOS_memory_allocate_p( NULL, size);
}

void gpOS_memory_deallocate( void *mem_ptr)
{
  gpOS_memory_deallocate_p( NULL, mem_ptr);
}

void memory_align( const unsigned int alignment)
{
  memory_align_p( NULL, alignment);
}

size_t gpOS_memory_getheapsize( void)
{
  return gpOS_memory_getheapsize_p( NULL);
}

size_t gpOS_memory_getheapfree( void)
{
  return gpOS_memory_getheapfree_p( NULL);
}

size_t gpOS_memory_getheaprequested( void)
{
  return gpOS_memory_getheaprequested_p( (gpOS_partition_t *)&os_root_partition);
}


gpOS_partition_t *gpOS_memory_create_partition( gpOS_memory_type_t type, void *start_ptr, const size_t size)
{
  gpOS_partition_t *new_partition = NULL;

  if( type == gpOS_MEMORY_TYPE_HEAP)
  {
    return NULL;
  }

  if( size == 0)
  {
    return NULL;
  }

  gpOS_memory_lock();

  new_partition = gpOS_memory_allocate_p( (gpOS_partition_t *)&os_root_partition, gpOS_MEMORY_ALIGN( sizeof( gpOS_partition_t), unsigned));

  if( new_partition != NULL)
  {
    // Enqueue new partition
    gpOS_partition_t *curr_part = &os_root_partition;
    gpOS_partition_t *prev_same = NULL;

    while( curr_part->next != NULL)
    {
      if( type == curr_part->type)
      {
        prev_same = curr_part;
      }
      curr_part = curr_part->next;
    }

    curr_part->next = new_partition;

    if( prev_same != NULL)
    {
      prev_same->next_same = new_partition;
    }

    new_partition->next       = NULL;
    new_partition->next_same  = NULL;
    new_partition->type       = type;
    new_partition->used       = FALSE;

    new_partition->start_ptr  = new_partition->curr_ptr   = start_ptr;
    new_partition->total_size = new_partition->curr_size  = size;
    new_partition->req_size   = 0;

    memory_align_p( new_partition, sizeof( unsigned));
  }

  gpOS_memory_unlock();

  return (gpOS_partition_t *)new_partition;
}

void *gpOS_memory_allocate_p( gpOS_partition_t *partition_ptr, const tSize size)
{
  void *result = NULL;
  gpOS_partition_t *memory_partition_ptr = (gpOS_partition_t *)partition_ptr;

  if( size == 0)
  {
    return NULL;
  }

  if( memory_partition_ptr == NULL)
  {
    memory_partition_ptr = &os_root_partition;
  }

  gpOS_memory_lock();

  if( memory_partition_ptr->type == gpOS_MEMORY_TYPE_SIMPLE)
  {
    if( size < memory_partition_ptr->curr_size)
    {
      result = (void *)memory_partition_ptr->curr_ptr;
      memory_partition_ptr->curr_ptr = (void *)((unsigned int)memory_partition_ptr->curr_ptr + size);

      memory_partition_ptr->curr_size -= size;

      memory_align_p( memory_partition_ptr, sizeof( unsigned));
    }
  }
  else if( memory_partition_ptr->type == gpOS_MEMORY_TYPE_HEAP)
  {
    // The heap extension is handled by libc implementation. So if partition type is heap
    // the memory allocator will use current used heap, not wanted one!
    result = _clibs_malloc( size);

  }

  // Update requested size if allocation was ok.
  if( result)
  {
    memory_partition_ptr->req_size = (unsigned)result - (unsigned)memory_partition_ptr->start_ptr + size;
  }

  gpOS_memory_unlock();

  return result;
}

void gpOS_memory_deallocate_p( gpOS_partition_t *partition_ptr, void *mem_ptr)
{
  gpOS_partition_t *memory_partition_ptr = (gpOS_partition_t *)partition_ptr;

  if( mem_ptr == NULL)
  {
    return;
  }

  if( memory_partition_ptr == NULL)
  {
    memory_partition_ptr = &os_root_partition;
  }

  gpOS_memory_lock();

  if( memory_partition_ptr->type == gpOS_MEMORY_TYPE_HEAP)
  {
    _clibs_free( mem_ptr);
  }

  gpOS_memory_unlock();

  return;
}

size_t gpOS_memory_getheapsize_p( gpOS_partition_t *partition_ptr)
{
  gpOS_partition_t *memory_partition_ptr = (gpOS_partition_t *)partition_ptr;

  if( memory_partition_ptr == NULL)
  {
    memory_partition_ptr = &os_root_partition;
  }

  return memory_partition_ptr->total_size;
}

size_t gpOS_memory_getheapfree_p( gpOS_partition_t *partition_ptr)
{
  gpOS_partition_t *memory_partition_ptr = (gpOS_partition_t *)partition_ptr;

  if( memory_partition_ptr == NULL)
  {
    memory_partition_ptr = &os_root_partition;
  }

  return memory_partition_ptr->total_size - memory_partition_ptr->req_size;
}

size_t gpOS_memory_getheaprequested_p( gpOS_partition_t *partition_ptr)
{
  gpOS_partition_t *memory_partition_ptr = (gpOS_partition_t *)partition_ptr;

  if( memory_partition_ptr == NULL)
  {
    memory_partition_ptr = &os_root_partition;
  }

  return memory_partition_ptr->req_size;
}

gpOS_partition_t *gpOS_memory_getnextpartition( gpOS_partition_t *part)
{
  if( part == NULL)
  {
    return os_root_partition.next;
  }
  else
  {
    return part->next;
  }
}

void gpOS_memory_lock( void)
{
  if( xmemory_mutex)
  {
    xSemaphoreTakeRecursive( xmemory_mutex, 0);
  }
}

void gpOS_memory_unlock( void)
{
  if( xmemory_mutex)
  {
    xSemaphoreGiveRecursive( xmemory_mutex);
  }
}
