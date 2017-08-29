/**
 * @file    freeRTOS_utils.h
 * @brief   FreeRTOS utilities handling definitions and macros.
 *
 */

#ifndef FREERTOS_UTILS_H
#define FREERTOS_UTILS_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "task.h"
#include "gpOS_types.h"
#include "portmacro.h"

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

extern taskTagCB_t *tagGetNext(taskTagCB_t *curTag);
extern UBaseType_t tagGetCPUUsage(taskTagCB_t *tag);
extern const char *tagGetName( taskTagCB_t *tag);

extern UBaseType_t get_tasks_status_table(TaskStatus_t ** const pxTaskStatusArray,
                                         uint32_t* const pulTotalRunTime );

extern UBaseType_t get_CPU_usage(void);

#endif /* FREERTOS_TIME_H */

