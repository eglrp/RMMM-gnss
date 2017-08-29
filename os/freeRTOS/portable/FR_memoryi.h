/**
 * @file    freeRTOS_memoryi.h
 * @brief   freeRTOS memory internal handling definitions and macros.
 *
 */

#ifndef FREERTOS_MEMORY_I_H
#define FREERTOS_MEMORY_I_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS_types.h"
#include "FR_memory.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
struct gpOS_partition_s
{
  gpOS_partition_t *    next;           /**< pointer to next partition */
  gpOS_partition_t *    next_same;      /**< pointer to next partition of same type, used for fast search */
  gpOS_memory_type_t    type;           /**< type of partition */
  void *              start_ptr;      /**< pointer to starting of memory partition */
  unsigned int        total_size;     /**< total size of memory partition */
  void *              curr_ptr;       /**< current pointer to free area of memory partition */
  unsigned int        curr_size;      /**< current free size in memory partition */
  unsigned int        req_size;       /**< current requested size in memory partition (for heap partition) */
  boolean_t           used;           /**< flags if heap type partition was already used or not */
} __attribute__ ((aligned));


/*****************************************************************************
   function prototypes
*****************************************************************************/
gpOS_partition_t *memgt_mallopt_set(gpOS_partition_t *);
gpOS_partition_t *memgt_mallopt_restore(gpOS_partition_t *);


void memory_align_p   ( gpOS_partition_t *custom_part, const unsigned int align_bytes);
void memory_align     ( const unsigned int align_bytes);

void vPortFree( void *pv );
void *pvPortMalloc( size_t xWantedSize );

#endif /* FREERTOS_MEMORY_I_H */


