/**
 * @file    o2f_mutex.c
 * @brief   OS mutexes wrapper to FreeRTOS
 *
 * @addtogroup gpOS_WRAPPER
 */


/*****************************************************************************
   includes
*****************************************************************************/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "FRi.h"
#include "FR_memoryi.h"
#include "FR_timei.h"

#include "gpOS_interrupt.h"

#ifdef FREERTOS_ITEM_STATUS
#include "OStoFR_mutex.h"
extern void  fis_add_mutex_item(char *file, int line, SemaphoreHandle_t fRtos_sem);
#else
#include "gpOS_mutex.h"
#endif

/*****************************************************************************
Create
*****************************************************************************/
#ifdef FREERTOS_ITEM_STATUS
gpOS_mutex_t *fis_mutex_create_p( char *file, int line, gpOS_partition_t *custom_part)
#else
gpOS_mutex_t *gpOS_mutex_create( gpOS_mutex_type_t param)
{
  return gpOS_mutex_create_p( param, NULL);
}

gpOS_mutex_t *gpOS_mutex_create_p( gpOS_mutex_type_t param, gpOS_partition_t *custom_part)
#endif
{
  gpOS_partition_t *origin_part;
  SemaphoreHandle_t fRTOSMutexHandle;

  // Make sure to allocate in correct partition
  origin_part = memgt_mallopt_set(custom_part);

  fRTOSMutexHandle = xSemaphoreCreateRecursiveMutex();

  // Restore Original partion
  memgt_mallopt_restore(origin_part);

#ifdef FREERTOS_ITEM_STATUS
  fis_add_mutex_item(file, line, fRTOSMutexHandle);
#endif

  return (gpOS_mutex_t *)fRTOSMutexHandle;
}

/*****************************************************************************
Delete
*****************************************************************************/
gpOS_error_t gpOS_mutex_delete( gpOS_mutex_t *os_mutex)
{
  gpOS_error_t rc;

  if(NULL == os_mutex) {
    rc = gpOS_FAILURE;
  } else {
    vSemaphoreDelete((SemaphoreHandle_t) os_mutex);
    rc = gpOS_SUCCESS;
  }

  return rc;
}

/*****************************************************************************
Lock / Take
*****************************************************************************/
void gpOS_mutex_lock( gpOS_mutex_t *os_mutex)
{
  SemaphoreHandle_t fRTOSMutexHandle = (SemaphoreHandle_t) os_mutex;

  xSemaphoreTakeRecursive(fRTOSMutexHandle, portMAX_DELAY);
}

/*****************************************************************************
TryLock / Immediate take / Take from ISR
*****************************************************************************/
gpOS_error_t gpOS_mutex_trylock( gpOS_mutex_t *os_mutex)
{
  SemaphoreHandle_t fRTOSMutexHandle = (SemaphoreHandle_t) os_mutex;
  BaseType_t success;

  if(interrupt_nested_count) {
    taskENTER_CRITICAL();
    success = xSemaphoreTakeFromISR(fRTOSMutexHandle, &xReschedule_fromISR_required);
    taskEXIT_CRITICAL();
  } else {
    success = xSemaphoreTakeRecursive(fRTOSMutexHandle, 0);
  }

  return (pdFALSE == success) ? gpOS_FAILURE: gpOS_SUCCESS ;
}

tUInt gpOS_mutex_locked( gpOS_mutex_t *os_mutex)
{
  return ((NULL == xSemaphoreGetMutexHolder((SemaphoreHandle_t)os_mutex)) ?  0 : 1);
}


/*****************************************************************************
Release/Give
*****************************************************************************/
gpOS_error_t gpOS_mutex_release( gpOS_mutex_t *os_mutex)
{
  SemaphoreHandle_t fRTOSMutexHandle = (SemaphoreHandle_t) os_mutex;
  BaseType_t success;

  if(interrupt_nested_count) {
    taskENTER_CRITICAL();
    success = xSemaphoreGiveFromISR(fRTOSMutexHandle, &xReschedule_fromISR_required);
    taskEXIT_CRITICAL();
  } else {
    success = xSemaphoreGiveRecursive(fRTOSMutexHandle);
  }

  return (pdFALSE == success) ? gpOS_FAILURE: gpOS_SUCCESS ;
}


