/**
 * @file    OStoFR_task.c
 * @brief   Wrapper OS - FreeRTOS task implementation.
 *
 * @addtogroup gpOS_WRAPPER
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "FreeRTOS.h"
#include "gpOS_task.h"
#include "FR_memoryi.h"
#include "FR_timei.h"
#include "portmacro.h"
#include "gpOStoFR_taski.h"
#include "gpOStoFR_utils.h"

/*****************************************************************************
   external declarations
*****************************************************************************/


/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define TASK_WITH_NO_NAME  "UNNAMED_TASK"

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

struct gpOS_task_s
{
  TaskHandle_t *FR_handle;
};

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/


/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/


/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/


/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
CREATION
*****************************************************************************/

gpOS_task_t *gpOS_task_create( gpOS_task_function_t proc_func, gpOS_task_param_t proc_param, const tSize stack_size, const tInt priority, const tChar *name, gpOS_task_flags_t flags)
{
  return gpOS_task_create_p( NULL, proc_func, proc_param, stack_size, priority, name, flags);
}

gpOS_task_t *gpOS_task_create_p( gpOS_partition_t* custom_part, gpOS_task_function_t proc_func, gpOS_task_param_t proc_param, const tSize stack_size, const tInt priority, const tChar *name, gpOS_task_flags_t flags)
{
  wrapper_task_t *wrapper_handle;
  TaskHandle_t freeRTOS_handle;
  gpOS_partition_t *origin_part;
  int create_priority;
  unsigned short stackDepth;

  // Task must not start. Create it with low prio.
  // Priority will be set once tag & suspend is okay
  create_priority = PRIO_FOR_CREATION_OF_SUSPENDED_TASK;

  O2F_W_ASSERT(priority > 1);

  wrapper_handle = (wrapper_task_t *)gpOS_memory_allocate_p(custom_part, sizeof(wrapper_task_t));

  if(wrapper_handle)
  {
    wrapper_handle->part = custom_part;
    stackDepth = stack_size/sizeof(StackType_t);

    // Make sure to allocate in correct partition
    origin_part = memgt_mallopt_set(custom_part);

    if(pdPASS != xTaskCreate((TaskFunction_t)proc_func,
                            name ? name : TASK_WITH_NO_NAME,
                            stackDepth,
                            (void *)proc_param,
                            create_priority,
                            &freeRTOS_handle))
    {
      // failed
      gpOS_memory_deallocate_p(custom_part, wrapper_handle);
      wrapper_handle = NULL;
    }

    // Restore Original partion
    memgt_mallopt_restore(origin_part);
  }

  // Creation Okay ?
  if(wrapper_handle) {
    taskTagCB_t *taskTag;

    if( flags == gpOS_TASK_FLAGS_SUSPENDED)
    {
      // Suspend the task and set its correct priority
      vTaskSuspend(freeRTOS_handle);
    }

    // Thanks to FreeRTOS Application Tag, the wrapper handle can be retrieve w/o
    // searching in the list of wrapper tasks.
    taskTag  = (taskTagCB_t *) xTaskGetApplicationTaskTag(freeRTOS_handle);
    taskTag->Tag2nd = (void *)wrapper_handle;

    // Wrapper job is done. Now set Priority.
    // Task may start before this function returns
    vTaskPrioritySet(freeRTOS_handle, tskIDLE_PRIORITY + priority);
  }

  return (gpOS_task_t *)freeRTOS_handle;
}

/*****************************************************************************

*****************************************************************************/
gpOS_error_t gpOS_task_delete( gpOS_task_t* os_handle)
{
  wrapper_task_t *wrapper_handle;
  TaskHandle_t freeRTOS_handle;
  gpOS_error_t rc;
  gpOS_partition_t *usedPartition;
  gpOS_partition_t *origin_part;
  taskTagCB_t *taskTag;

  if(NULL == os_handle) {
    rc = gpOS_SUCCESS;  // Yes, os return success when input is NULL
  }
  else
  {
    freeRTOS_handle = (TaskHandle_t)os_handle;

    taskTag  = (taskTagCB_t *) xTaskGetApplicationTaskTag(freeRTOS_handle);
    wrapper_handle = (struct task_s *)(taskTag->Tag2nd);

    if(NULL == wrapper_handle) {
      rc = gpOS_FAILURE;
    } else {
#if( INCLUDE_vTaskDelete == 1 )
      // Delete freeRTOS task
      vTaskDelete(freeRTOS_handle);
#else
      vTaskSuspend(freeRTOS_handle);
#endif
      usedPartition = wrapper_handle->part;

      // Delete wrapper context
      gpOS_memory_deallocate_p(usedPartition, wrapper_handle);

      // Make sure OS uses in correct partition
      origin_part = memgt_mallopt_set(usedPartition);

      // Restore Original partion
      memgt_mallopt_restore(origin_part);

      rc = gpOS_SUCCESS;
    }
  }

  return rc;
}

/*****************************************************************************

*****************************************************************************/
void gpOS_task_delay(const gpOS_clock_t delay)
{
 taskHiResDelay((gpOS_clock_t)(gpOS_time_now() + delay));
}

/*****************************************************************************

*****************************************************************************/
void gpOS_task_delay_until( const gpOS_clock_t timeout)
{
  //vTaskDelay(OStoFR_ticks(timeout)); WOULD BE LOW RESOLUTION

  taskHiResDelay((gpOS_clock_t) timeout);
}

/*****************************************************************************

*****************************************************************************/
gpOS_error_t gpOS_task_suspend( gpOS_task_t* os_handle)
{
  vTaskSuspend((TaskHandle_t)os_handle);

  return gpOS_SUCCESS;
}

/*****************************************************************************

*****************************************************************************/
gpOS_error_t gpOS_task_resume( gpOS_task_t* os_handle)
{
  vTaskResume((TaskHandle_t)os_handle);

  return gpOS_SUCCESS;
}

/*****************************************************************************

*****************************************************************************/
gpOS_task_t *gpOS_task_get_head( void)
{
  return (gpOS_task_t *)NULL;
}

/*****************************************************************************

*****************************************************************************/
gpOS_task_t *gpOS_task_get_next( gpOS_task_t *task)
{
  return (gpOS_task_t *)NULL;
}

/*****************************************************************************

*****************************************************************************/
gpOS_ISR gpOS_task_t *gpOS_task_get_id( void)
{
  return (gpOS_task_t *)xTaskGetCurrentTaskHandle();
}

/*****************************************************************************

*****************************************************************************/
int gpOS_task_set_priority( gpOS_task_t* os_handle, const int new_priority)
{
  TaskHandle_t freeRTOS_handle;
  UBaseType_t  old_prio;

  freeRTOS_handle = (TaskHandle_t)os_handle;

  old_prio = uxTaskPriorityGet(freeRTOS_handle);

  vTaskPrioritySet(freeRTOS_handle, tskIDLE_PRIORITY + new_priority);

  return (int) old_prio;
}

/*****************************************************************************

*****************************************************************************/
int gpOS_task_get_priority( gpOS_task_t* os_handle)
{
  TaskHandle_t freeRTOS_handle;
  UBaseType_t  prio;

  freeRTOS_handle = (TaskHandle_t)os_handle;

  prio = uxTaskPriorityGet(freeRTOS_handle);

  return (int) (prio - tskIDLE_PRIORITY);
}

/*****************************************************************************

*****************************************************************************/
void *gpOS_task_get_stack_base( gpOS_task_t* os_handle)
{
  return NULL;
}

/*****************************************************************************

*****************************************************************************/
void *gpOS_task_get_stack_ptr( gpOS_task_t* os_handle)
{
  return NULL;
}

/*****************************************************************************

*****************************************************************************/
size_t gpOS_task_get_stack_size( gpOS_task_t* os_handle)
{
  return 0;
}

/*****************************************************************************

*****************************************************************************/
size_t gpOS_task_get_stack_used( gpOS_task_t* os_handle)
{
  UBaseType_t max_depth;

  max_depth = uxTaskGetStackHighWaterMark((TaskHandle_t)os_handle);

  return (size_t)(max_depth * sizeof(StackType_t));
}

/*****************************************************************************

*****************************************************************************/
tUInt gpOS_task_get_cpuusage( gpOS_task_t* task)
{
  return 0;
}

/*****************************************************************************

*****************************************************************************/
const char *gpOS_task_get_name( gpOS_task_t* os_handle)
{
  return (const char *)pcTaskGetTaskName((TaskHandle_t)os_handle);
}

/*****************************************************************************

*****************************************************************************/
void gpOS_task_lock( void)
{
  vTaskSuspendAll();
}

/*****************************************************************************

*****************************************************************************/
void gpOS_task_unlock( void)
{
  xTaskResumeAll();
}

