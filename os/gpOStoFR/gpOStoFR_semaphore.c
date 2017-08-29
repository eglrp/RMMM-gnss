/**
 * @file    OStoFR_semaphore.c
 * @brief   Wrapper OS - FreeRTOS semaphore implementation.
 *
 * @addtogroup gpOS_WRAPPER
 */

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "FRi.h"
#include "FR_memoryi.h"
#include "FR_timei.h"

#include "gpOS_interrupt.h"
#include "gpOStoFR_utils.h"

#ifdef FREERTOS_ITEM_STATUS
#include "gpOStoFR_semaphore.h"
extern void  fis_sem_add_item(char *file, int line, SemaphoreHandle_t fRtos_sem);
#else
#include "gpOS_semaphore.h"
#endif

#ifdef FREERTOS_CHECK_SEMAPHORES
#include "gnss_debug.h"
#include "lld_uart.h"
extern int set_trace;
extern volatile void * volatile pxCurrentTCB;
extern void vFreeRTOS_Abort(void);

typedef struct gnss_debug_manager_s
{
  char                   string[512];
  gpOS_semaphore_t *          access_sem;
  gnss_debug_writeout_t  write;
} gnss_debug_manager_t;

typedef struct svc_uart_fifo_handler_s {
  gpOS_semaphore_t * access_sem;
  gpOS_semaphore_t * irq_sem;
  boolean_t     not_empty_waiting;
  boolean_t     not_full_waiting;
  tU8 *         buf_ptr;
  tU8 *         in_ptr;
  tU8 *         out_ptr;
  tU16          buf_size;
  tU16          len;
} svc_uart_fifo_handler_t;

typedef struct svc_uart_port_handler_s svc_uart_port_handler_t;

struct svc_uart_port_handler_s
{
  svc_uart_port_handler_t *next;
  boolean_t                open;
  gpOS_mutex_t *             access_mutex;
  unsigned                 id;
  LLD_UART_BaudRateTy      baud_rate;
  svc_uart_fifo_handler_t  fifo_rx;
  svc_uart_fifo_handler_t  fifo_tx;
};

typedef struct svc_uart_handler_s
{
  gpOS_partition_t *               partition;
  tU32                        mem_used;
  tU32                        bus_speed;
  tU8                         ports;
  svc_uart_port_handler_t *  port_head;
} svc_uart_handler_t;

extern gnss_debug_manager_t *gnss_debug_manager;
extern gpOS_semaphore_t *crt_reent_sem;
extern svc_uart_handler_t *svc_uart_handler;

typedef enum
{
SEM_WAIT,
SEM_SIG
} sem_state;

typedef struct sem_saved_s
{
  sem_state state;
  gpOS_semaphore_t *sem;
  void *task;
  int irq;
} sem_saved_t;

#define   SEM_SAVED_MAX 100
sem_saved_t sem_saved_table[SEM_SAVED_MAX];
int sem_saved_Id = 0;
int sem_saved_Count = 0;
sem_state sem_LastSaved_state = 2;

void vDo_save_semList(sem_state state, gpOS_semaphore_t *sem, int irq_status)
{
  int *ptr = svc_uart_handler;
  if ( ( set_trace == 1) &&
       (sem != gnss_debug_manager->access_sem) &&
       (sem != crt_reent_sem) &&
       (sem != *(ptr+0xD)) &&
       (sem != *(ptr+0xE)) &&
       (sem != *(ptr+0x14)) &&
       (sem != *(ptr+0x15))
     )
  {
    if (sem_saved_Id >= SEM_SAVED_MAX) sem_saved_Id=0;
    sem_saved_table[sem_saved_Id].state = state;
    sem_saved_table[sem_saved_Id].sem = sem;
    sem_saved_table[sem_saved_Id].task = pxCurrentTCB;
    sem_saved_table[sem_saved_Id].irq = irq_status;

    if(sem_LastSaved_state == state)
      sem_saved_Count++;
    else
      sem_saved_Count=0;

    sem_LastSaved_state = state;
    sem_saved_Id++;
    //GPS_DEBUG_MSG(( ">>>> [sem] wait %x\r\n", sem));
  }

  if (sem_saved_Count > (SEM_SAVED_MAX>>2) ) set_trace = 2; //vFreeRTOS_Abort();
}
#endif

/************************************************************************
Creation
************************************************************************/
#ifdef FREERTOS_ITEM_STATUS
gpOS_semaphore_t *fis_semaphore_create_p( char *file, int line, gpOS_partition_t *custom_part, const int start_value)
#else
gpOS_semaphore_t *gpOS_semaphore_create( gpOS_sem_type_t sem_param, const int start_value)
{
  return gpOS_semaphore_create_p( sem_param, NULL, start_value);
}

gpOS_semaphore_t *gpOS_semaphore_create_p( gpOS_sem_type_t sem_param, gpOS_partition_t *custom_part, const int start_value)
#endif
{
  SemaphoreHandle_t fRTOSSemHandle;
  UBaseType_t maxCount = 0x7FFFFFFF; // OS has no max count
  gpOS_partition_t *origin_part;

  // Make sure to allocate in correct partition
  origin_part = memgt_mallopt_set(custom_part);

  fRTOSSemHandle = xSemaphoreCreateCounting(maxCount, start_value);

  // Restore Original partion
  memgt_mallopt_restore(origin_part);

#ifdef FREERTOS_ITEM_STATUS
  fis_sem_add_item(file, line, fRTOSSemHandle);
#endif

  return (gpOS_semaphore_t *)fRTOSSemHandle;
}

/************************************************************************
Delete
************************************************************************/
gpOS_error_t gpOS_semaphore_delete( gpOS_semaphore_t *sem)
{
  gpOS_error_t rc;

  if(NULL == sem) {
    rc = gpOS_FAILURE;
  } else {
    vSemaphoreDelete((SemaphoreHandle_t) sem);
    rc = gpOS_SUCCESS;
  }

  return rc;
}

/************************************************************************
Wait  / Take
************************************************************************/
void gpOS_semaphore_wait(gpOS_semaphore_t *sem)
{
  gpOS_semaphore_wait_timeout( sem, gpOS_TIMEOUT_INFINITY);
}

gpOS_error_t gpOS_semaphore_wait_timeout(gpOS_semaphore_t *sem, const gpOS_clock_t *timeout)
{
  SemaphoreHandle_t fRTOSSemHandle = (SemaphoreHandle_t) sem;
  BaseType_t success;

  if(interrupt_nested_count) {
    taskENTER_CRITICAL();
    success = xSemaphoreTakeFromISR(fRTOSSemHandle, &xReschedule_fromISR_required);
    taskEXIT_CRITICAL();
  } else {
    success = xSemaphoreTake(fRTOSSemHandle, OStoFR_timeout(timeout));
  }

  #ifdef FREERTOS_CHECK_SEMAPHORES
  vDo_save_semList(SEM_WAIT, sem, interrupt_nested_count);
  #endif

  return (pdFALSE == success) ? gpOS_FAILURE: gpOS_SUCCESS ;
}

/************************************************************************
Signal / Give
************************************************************************/
gpOS_ISR void gpOS_semaphore_signal              ( gpOS_semaphore_t *sem)
{
  SemaphoreHandle_t fRTOSSemHandle = (SemaphoreHandle_t) sem;

  if(interrupt_nested_count) {
    taskENTER_CRITICAL();
    xSemaphoreGiveFromISR(fRTOSSemHandle, &xReschedule_fromISR_required);
    taskEXIT_CRITICAL();
  } else {
    xSemaphoreGive(fRTOSSemHandle);
  }

  #ifdef FREERTOS_CHECK_SEMAPHORES
  vDo_save_semList(SEM_SIG, sem, interrupt_nested_count);
  #endif
}

/************************************************************************
Value / Count

This function should belong to freeRTOS.
In version 8.0.0, FreeRTOS semaphores relies in queues directly.
Semaphore handle is same than queue handle.

But count access API is not available for semaphores. Here matching is made
************************************************************************/
gpOS_ISR int gpOS_semaphore_value( gpOS_semaphore_t *sem)
{
  QueueHandle_t underlying_queue = (QueueHandle_t) sem;
  UBaseType_t fRtosCount;

  if(interrupt_nested_count) {
    taskENTER_CRITICAL();
    fRtosCount = uxQueueMessagesWaitingFromISR(underlying_queue);
    taskEXIT_CRITICAL();
  } else {
    fRtosCount = uxQueueMessagesWaiting(underlying_queue);
  }

  return (int) fRtosCount;
}



