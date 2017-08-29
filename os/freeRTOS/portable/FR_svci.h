/**
 * @file    freeRTOS_svci.h
 * @brief   FreeRTOS time internal handling definitions and macros.
 *
 */

#ifndef FREERTOS_SVCI_H
#define FREERTOS_SVCI_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "FR_memoryi.h"

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

extern unsigned xOS_stats_get_interrupts_occurred_number(void);
extern unsigned xOS_stats_get_context_switch_number(void);
extern void     vOS_vectors_stack_init( gpOS_partition_t *part);
extern void     vOS_stats_init( void);

#endif /* FREERTOS_SVCI_H */
