
/**
 * @file    freeRTOS_utils.c
 * @brief   FreeRTOS utilities handling implementation.
 *
 */

/*****************************************************************************
   includes
*****************************************************************************/

/*{{{  include*/
#include "FreeRTOS.h"
#include "FR_memory.h"
#include "FR_utils.h"
#include "gpOS_time.h"
#include "task.h"
#include "gnss_debug.h"

/*}}}  */

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
static uint8_t      uNbTasksStatsEntries = 20; // To be retrieved from somewhere...
taskTagCB_t *tagList = NULL;
taskTagCB_t *prevTaskTag = NULL;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
osTimeStats_t xOSStats;
#endif

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/************************************************************************
Status table retrieval with customized runtime data
************************************************************************/
UBaseType_t uxTaskGetSystemStateCustom(TaskStatus_t *const pxTaskStatusArray,
                                      const UBaseType_t uxArraySize,
                                      uint32_t * const pulTotalRunTime )
{
  UBaseType_t nb_task_reported;
#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
  taskTagCB_t *thisTaskTagPtr;
	UBaseType_t t_idx;

  taskENTER_CRITICAL();
#endif // configGENERATE_RUN_TIME_STATSCustom

  nb_task_reported = uxTaskGetSystemState(pxTaskStatusArray,
                                          uxArraySize,
                                          NULL);

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
  if(NULL != pulTotalRunTime) {
    *pulTotalRunTime = xOSStats.prevPeriodDuration;
  }

  t_idx=0;
  while(t_idx < nb_task_reported)
  {
    thisTaskTagPtr = (taskTagCB_t *)xTaskGetApplicationTaskTag(pxTaskStatusArray[t_idx].xHandle);

    pxTaskStatusArray[t_idx].ulRunTimeCounter = thisTaskTagPtr->prevPeriodSum;
    t_idx++;
  }

  taskEXIT_CRITICAL();
#endif

  return nb_task_reported;
}

/************************************************************************
Status table allocation/retrieval
************************************************************************/
static TaskStatus_t * get_status_table_ptr()
{
  static TaskStatus_t *pTaskStatusTable = NULL;

  if(NULL == pTaskStatusTable) {
    pTaskStatusTable = (TaskStatus_t *)gpOS_memory_allocate(uNbTasksStatsEntries * sizeof(TaskStatus_t));
  }

  return pTaskStatusTable;
}

/************************************************************************
Systeme Status table
************************************************************************/
UBaseType_t get_tasks_status_table(TaskStatus_t ** const pxTaskStatusArray,
                                   uint32_t* const pulTotalRunTime )
{
  UBaseType_t nb_task_reported;

  *pxTaskStatusArray = get_status_table_ptr();

  if(NULL == *pxTaskStatusArray) {
    nb_task_reported = 0;
  } else {
    nb_task_reported = uxTaskGetSystemStateCustom(*pxTaskStatusArray,
                                                  uNbTasksStatsEntries,
                                                  pulTotalRunTime);
  }

  return nb_task_reported;
}

/************************************************************************
Allow to scan the tagList
************************************************************************/
taskTagCB_t *tagGetNext(taskTagCB_t *curTag)
{
  if(NULL == curTag) {
    curTag = tagList;
  } else {
    curTag = curTag->next;
  }
  return curTag;
}

/************************************************************************
Return task name for this tag
************************************************************************/
const char *tagGetName( taskTagCB_t *tag)
{
  return tag->tName;
}

/************************************************************************
Compute task CPU Usage. Reports 10000 when 100.00%, 5000 when 50.00% ...
************************************************************************/
UBaseType_t tagGetCPUUsage(taskTagCB_t *tag)
{
  UBaseType_t usage;

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )

  uint32_t taskPrevDuration;
  uint32_t prevPeriodDuration;

  if(NULL == tag) {
    usage = 0;
  } else {
    taskENTER_CRITICAL();
    taskPrevDuration = tag->prevPeriodSum;
    prevPeriodDuration = xOSStats.prevPeriodDuration;
    taskEXIT_CRITICAL();

    // Check validity
    if(0 != prevPeriodDuration) {
      usage = (UBaseType_t) ((float)taskPrevDuration / (float)prevPeriodDuration * 10000);
    } else {
      usage = 0;
    }
  }
#else
  usage = 0;
#endif

  return usage;
}

/************************************************************************
Compute CPU Usage. Reports 10000 when 100.00%, 5000 when 50.00% ...
************************************************************************/
UBaseType_t get_CPU_usage(void)
{
  UBaseType_t usage;

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
  static taskTagCB_t *idleTagPtr = NULL;

  if(NULL == idleTagPtr) {
    TaskHandle_t idleHandle;

    idleHandle = xTaskGetIdleTaskHandle();
    idleTagPtr = (taskTagCB_t *)xTaskGetApplicationTaskTag(idleHandle);
  }

  usage = 10000 - tagGetCPUUsage(idleTagPtr);
#else
  usage = 0;
#endif

  return usage;
}

/************************************************************************
Get tasks CPU Usage for complete system
************************************************************************/
UBaseType_t get_tasks_cpu_table(TaskStatus_t ** const pxTaskStatusArray,
                             unsigned long * const pulTotalRunTime )
{
  UBaseType_t nb_task_reported = 0;

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
  taskTagCB_t *thisTaskTagPtr = NULL;
#endif // configGENERATE_RUN_TIME_STATSCustom

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
  taskENTER_CRITICAL();

  *pxTaskStatusArray = get_status_table_ptr();

  if(NULL != *pxTaskStatusArray) {
    if(NULL != pulTotalRunTime) {
      *pulTotalRunTime = xOSStats.prevPeriodDuration;
    }

    thisTaskTagPtr = tagGetNext(thisTaskTagPtr);
    while(thisTaskTagPtr) {
      (*pxTaskStatusArray)[nb_task_reported].ulRunTimeCounter = thisTaskTagPtr->prevPeriodSum;
      (*pxTaskStatusArray)[nb_task_reported].pcTaskName = thisTaskTagPtr->tName;
      nb_task_reported++;

      thisTaskTagPtr = tagGetNext(thisTaskTagPtr);
    }

    taskEXIT_CRITICAL();
  }
#endif // configGENERATE_RUN_TIME_STATSCustom

  return nb_task_reported;
}

#define INLINE_ASM_PORT_ENTER \
    "\
    .balign 4\n                   ;\
    push {r3}                     ;\
    nop                           ;\
    mov   r3, pc                  ;\
    bx    r3                      ;\
    .arm                          ;\
    ldmfd sp!, {r3}               ;\
    "
#define INLINE_ASM_PORT_EXIT \
    "\
    .arm\n                        ;\
    stmfd sp!, {r3}               ;\
    add   r3, pc, #1              ;\
    bx    r3                      ;\
    .thumb                        ;\
    pop {r3}                      ;\
    nop                           ;\
    " \
    : \
    : \
    : "r3"

#define FREERTOS_ABORT()             \
{																\
	__asm volatile (							\
 				INLINE_ASM_PORT_ENTER   \
				"b 0x04						\n\t" \
  			INLINE_ASM_PORT_EXIT    \
  );                            \
}

void vFreeRTOS_Abort(void)
{
  portENTER_CRITICAL();
  #if defined( __GNUC__)
  FREERTOS_ABORT();
  #else
  #if defined( __ARMCC_VERSION)
  #pragma arm
  __asm
  {
    b 0x04
  }
  #endif
  #endif
}
