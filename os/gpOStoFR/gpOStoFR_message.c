/**
 * @file    FRtoOS_message.c
 * @brief   Wrapper OS - FreeRTOS message implementation.
 *
 * @addtogroup gpOS_WRAPPER
 */

#include "FreeRTOS.h"
#include "FR_memoryi.h"
#include "FR_timei.h"
#include "FRi.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "gpOStoFR_messagei.h"
#include "gpOStoFR_utils.h"
#include "gpOS_time.h"
#include "gpOS_interrupt.h"

#if defined( FREERTOS_ITEM_STATUS)
#define NB_SEM_MAX 70
#include "list.h"
#include "clibs.h"
#include "FreeRTOSConfig.h"
#include "FRtoOS_message.h"

typedef struct QueueDefinition
{
	int8_t *pcHead;					/*< Points to the beginning of the queue storage area. */
	int8_t *pcTail;					/*< Points to the byte at the end of the queue storage area.  Once more byte is allocated than necessary to store the queue items, this is used as a marker. */
	int8_t *pcWriteTo;				/*< Points to the free next place in the storage area. */

	union							/* Use of a union is an exception to the coding standard to ensure two mutually exclusive structure members don't appear simultaneously (wasting RAM). */
	{
		int8_t *pcReadFrom;			/*< Points to the last place that a queued item was read from when the structure is used as a queue. */
		UBaseType_t uxRecursiveCallCount;/*< Maintains a count of the number of times a recursive mutex has been recursively 'taken' when the structure is used as a mutex. */
	} u;

	List_t xTasksWaitingToSend;		/*< List of tasks that are blocked waiting to post onto this queue.  Stored in priority order. */
	List_t xTasksWaitingToReceive;	/*< List of tasks that are blocked waiting to read from this queue.  Stored in priority order. */

	volatile UBaseType_t uxMessagesWaiting;/*< The number of items currently in the queue. */
	UBaseType_t uxLength;			/*< The length of the queue defined as the number of items it will hold, not the number of bytes. */
	UBaseType_t uxItemSize;			/*< The size of each items that the queue will hold. */

	volatile BaseType_t xRxLock;	/*< Stores the number of items received from the queue (removed from the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */
	volatile BaseType_t xTxLock;	/*< Stores the number of items transmitted to the queue (added to the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */

	#if ( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t uxQueueNumber;
		uint8_t ucQueueType;
	#endif

	#if ( configUSE_QUEUE_SETS == 1 )
		struct QueueDefinition *pxQueueSetContainer;
	#endif

} xQUEUE;

/* The old xQUEUE name is maintained above then typedefed to the new Queue_t
name below to enable the use of older kernel aware debuggers. */
typedef xQUEUE Queue_t;

typedef struct tskTaskControlBlock
{
	volatile StackType_t	*pxTopOfStack;	/*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

	#if ( portUSING_MPU_WRAPPERS == 1 )
		xMPU_SETTINGS	xMPUSettings;		/*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE TCB STRUCT. */
	#endif

	ListItem_t			xGenericListItem;	/*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
	ListItem_t			xEventListItem;		/*< Used to reference a task from an event list. */
	UBaseType_t			uxPriority;			/*< The priority of the task.  0 is the lowest priority. */
	StackType_t			*pxStack;			/*< Points to the start of the stack. */
	char				pcTaskName[ configMAX_TASK_NAME_LEN ];/*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

	#if ( portSTACK_GROWTH > 0 )
		StackType_t		*pxEndOfStack;		/*< Points to the end of the stack on architectures where the stack grows up from low memory. */
	#endif

	#if ( portCRITICAL_NESTING_IN_TCB == 1 )
		UBaseType_t 	uxCriticalNesting; 	/*< Holds the critical section nesting depth for ports that do not maintain their own count in the port layer. */
	#endif

	#if ( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t		uxTCBNumber;		/*< Stores a number that increments each time a TCB is created.  It allows debuggers to determine when a task has been deleted and then recreated. */
		UBaseType_t  	uxTaskNumber;		/*< Stores a number specifically for use by third party trace code. */
	#endif

	#if ( configUSE_MUTEXES == 1 )
		UBaseType_t 	uxBasePriority;		/*< The priority last assigned to the task - used by the priority inheritance mechanism. */
	#endif

	#if ( configUSE_APPLICATION_TASK_TAG == 1 )
		TaskHookFunction_t pxTaskTag;
	#endif

	#if ( configGENERATE_RUN_TIME_STATS == 1 )
		uint32_t		ulRunTimeCounter;	/*< Stores the amount of time the task has spent in the Running state. */
	#endif

	#if ( configUSE_NEWLIB_REENTRANT == 1 )
		/* Allocate a Newlib reent structure that is specific to this task.
		Note Newlib support has been included by popular demand, but is not
		used by the FreeRTOS maintainers themselves.  FreeRTOS is not
		responsible for resulting newlib operation.  User must be familiar with
		newlib and must provide system-wide implementations of the necessary
		stubs. Be warned that (at the time of writing) the current newlib design
		implements a system-wide malloc() that must be provided with locks. */
		struct 	_reent xNewLib_reent;
	#endif

} tskTCB;

/* The old tskTCB name is maintained above then typedefed to the new TCB_t name
below to enable the use of older kernel aware debuggers. */
typedef tskTCB TCB_t;

struct fis_sem_item {
  char file[60];
  int line;
  SemaphoreHandle_t fRtos_sem;
};

struct fis_sem_item fis_sem_item_table[NB_SEM_MAX];
int fis_sem_item_idx = 0;

void  fis_sem_add_item(char *file, int line, SemaphoreHandle_t fRtos_sem)
{
  if(strlen(file) > 59) while(1) {}

  strcpy(fis_sem_item_table[fis_sem_item_idx].file, file);
  fis_sem_item_table[fis_sem_item_idx].line = line;
  fis_sem_item_table[fis_sem_item_idx].fRtos_sem = fRtos_sem;

  fis_sem_item_idx++;
  if(fis_sem_item_idx >= NB_SEM_MAX) while(1) {}
}

int fis_sem_status(char *out_msg, int item_idx, int verbose)
{
  int rc;
  int index = 0;
  Queue_t *zeQueue;

  if(item_idx >= fis_sem_item_idx) {
    rc = 0;
  } else {
    rc = 1;
    zeQueue = (Queue_t *)fis_sem_item_table[item_idx].fRtos_sem;

    index = _clibs_sprintf(out_msg, "$PSTMFISSEM,0x%08X,%04d,%s",
         fis_sem_item_table[item_idx].fRtos_sem,
         fis_sem_item_table[item_idx].line,
         fis_sem_item_table[item_idx].file
         );
    index += _clibs_sprintf( out_msg + index, "*%02x\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );

    if(verbose) {
      if(zeQueue->xTasksWaitingToReceive.uxNumberOfItems != 0) {
          index = _clibs_sprintf(out_msg, "$PSTMFISSEMWAITERS,%d",
                       zeQueue->xTasksWaitingToReceive.uxNumberOfItems);
          index += _clibs_sprintf( out_msg + index, "*%02x\r\n", nmea_support_checksum( out_msg ) );
          nmea_send_msg_to_uart( out_msg, index );
      }
    }
  }
  return rc;
}

struct fis_mutex_item {
  char file[60];
  int line;
  SemaphoreHandle_t fRtos_mutex;
};

struct fis_mutex_item fis_mutex_item_table[30];
int fis_mutex_item_idx = 0;

void  fis_add_mutex_item(char *file, int line, SemaphoreHandle_t fRtos_mutex)
{
  strcpy(fis_mutex_item_table[fis_mutex_item_idx].file, file);
  fis_mutex_item_table[fis_mutex_item_idx].line = line;
  fis_mutex_item_table[fis_mutex_item_idx].fRtos_mutex = fRtos_mutex;

  fis_mutex_item_idx++;
}

int fis_mutex_status(char *out_msg, int item_idx, int verbose)
{
  int rc;
  int index = 0;
  Queue_t *zeQueue;

  if(item_idx >= fis_mutex_item_idx) {
    rc = 0;
  } else {
    rc = 1;
    zeQueue = (Queue_t *)fis_mutex_item_table[item_idx].fRtos_mutex;

    index = _clibs_sprintf(out_msg, "$PSTMFISMUTEX,0x%08X,%04d,%s",
         fis_mutex_item_table[item_idx].fRtos_mutex,
         fis_mutex_item_table[item_idx].line,
         fis_mutex_item_table[item_idx].file
         );
    index += _clibs_sprintf( out_msg + index, "*%02x\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );

    if(verbose) {
      if(zeQueue->xTasksWaitingToReceive.uxNumberOfItems != 0) {
          index = _clibs_sprintf(out_msg, "$PSTMFISMUTEXWAITERS,%d",
                       zeQueue->xTasksWaitingToReceive.uxNumberOfItems);
          index += _clibs_sprintf( out_msg + index, "*%02x\r\n", nmea_support_checksum( out_msg ) );
          nmea_send_msg_to_uart( out_msg, index );
      }
    }
  }
  return rc;
}

struct fis_queue_item {
  char file[60];
  int line;
  gpOS_message_queue_t *w_queue;
  QueueHandle_t fRtos_queue;
  SemaphoreHandle_t sem;
};

struct fis_queue_item fis_queue_item_table[30];
int fis_queue_item_idx = 0;

static void  fis_add_queue_item(char *file, int line, gpOS_message_queue_t *w_queue, QueueHandle_t fRtos_queue, SemaphoreHandle_t sem)
{
  strcpy(fis_queue_item_table[fis_queue_item_idx].file, file);
  fis_queue_item_table[fis_queue_item_idx].line = line;
  fis_queue_item_table[fis_queue_item_idx].w_queue = w_queue;
  fis_queue_item_table[fis_queue_item_idx].fRtos_queue = fRtos_queue;
  fis_queue_item_table[fis_queue_item_idx].sem = sem;

  fis_queue_item_idx++;
}
int fis_queue_status(char *out_msg, int item_idx, int verbose)
{
  int rc;
  int index = 0;
  Queue_t *zeQueue;
  if(item_idx >= fis_queue_item_idx) {
    rc = 0;
  } else {
    zeQueue = (Queue_t *)fis_queue_item_table[item_idx].fRtos_queue;
    rc = 1;

    index = _clibs_sprintf(out_msg, "$PSTMFISQUEUE,0x%08X,0x%08X,0x%08X,%04d,%s",
           fis_queue_item_table[item_idx].w_queue,
           fis_queue_item_table[item_idx].fRtos_queue,
           fis_queue_item_table[item_idx].sem,
           fis_queue_item_table[item_idx].line,
           fis_queue_item_table[item_idx].file
           );
    index += _clibs_sprintf( out_msg + index, "*%02x\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );

    if(verbose) {
      TCB_t *taskPtr;
      if(zeQueue->xTasksWaitingToReceive.uxNumberOfItems != 0) {
          index = _clibs_sprintf(out_msg, "$PSTMFISQUEUEWAITRCV,%d",
                       zeQueue->xTasksWaitingToReceive.uxNumberOfItems);
          taskPtr =  (TCB_t *)(zeQueue->xTasksWaitingToReceive.xListEnd.pxNext->pvOwner);
          index += _clibs_sprintf(out_msg+index, ",0x%08X,%s",
                       taskPtr,
                       taskPtr->pcTaskName);
          index += _clibs_sprintf( out_msg + index, "*%02x\r\n", nmea_support_checksum( out_msg ) );
          nmea_send_msg_to_uart( out_msg, index );
      }
    }
  }
  return rc;
}
#else
#include "gpOS_message.h"
#endif

/************************************************************************
Queue Creation
************************************************************************/
#ifdef FREERTOS_ITEM_STATUS
gpOS_message_queue_t* fis_message_create_queue( tChar *file, tUInt line, tSize body_size, tUInt num_of_msgs)
{
  return fis_message_create_queue_p( file, line, NULL, body_size, num_of_msgs);
}
#else
gpOS_message_queue_t* gpOS_message_create_queue( tSize body_size, tUInt num_of_msgs)
{
  return gpOS_message_create_queue_p( NULL, body_size, num_of_msgs);
}
#endif

/************************************************************************
Queue Creation using specific partition
************************************************************************/
#ifdef FREERTOS_ITEM_STATUS
gpOS_message_queue_t *fis_message_create_queue_p( tChar *file, tUInt line, gpOS_partition_t *custom_part, tSize msg_body_size, tUInt num_of_msgs)
#else
gpOS_message_queue_t *gpOS_message_create_queue_p( gpOS_partition_t *custom_part, tSize msg_body_size, tUInt num_of_msgs)
#endif
{
  gpOS_partition_t *origin_part;
  gpOS_message_queue_t *w_queue;
  char *bufs;
  size_t aligned_msg_body_size;

  if((0 == msg_body_size) || (0 == num_of_msgs)) {
    w_queue = NULL;
  } else {
    // Allocate control and memory blocks. Manage to have them aligned on 4 bytes.
    // Allocating a single memory space avoid to remember where de memory blocks are.
    aligned_msg_body_size = (msg_body_size+3) & 0xFFFFFFFC;
    w_queue = (gpOS_message_queue_t *)gpOS_memory_allocate_p(custom_part,
                    sizeof(gpOS_message_queue_t) + (aligned_msg_body_size * num_of_msgs));
    if(w_queue) {
      w_queue->part = custom_part;

      // Make sure OS allocates in correct partition
      origin_part = memgt_mallopt_set(custom_part);

      // Create FreeRTOS queue holds only ptr to our buffers
      // Add 1 message for HiResolution Timeout receival
      w_queue->fRtos_queue = xQueueCreate(num_of_msgs + 1, sizeof(void *));

      if(!w_queue->fRtos_queue) {
        gpOS_memory_deallocate_p(custom_part, w_queue);
        w_queue = NULL;
      } else {
        w_queue->cnt_sem = xSemaphoreCreateCounting(num_of_msgs, 0);
        w_queue->TimeOutMessage = TIME_OUT_IDENTIFIER;

        if(!w_queue->cnt_sem) {
          vQueueDelete(w_queue->fRtos_queue);
          gpOS_memory_deallocate_p(custom_part, w_queue);
          w_queue = NULL;
        }
      }
      // Restore Original partion
      memgt_mallopt_restore(origin_part);
    }
  }


  if(w_queue) {
    // All is correct. Chain blocks and make them avalable
    w_queue->free_blocks = NULL;

    // Add buffers to available list (allocated just after CB)
    bufs = (char *)&(w_queue[1]);

    while(num_of_msgs) {
      gpOS_message_release(w_queue, (void *)bufs);
      bufs += aligned_msg_body_size;
      num_of_msgs--;
    }

#ifdef FREERTOS_ITEM_STATUS
    fis_add_queue_item(file, line, w_queue, w_queue->fRtos_queue, w_queue->cnt_sem);
#endif

  }

  return w_queue;
}

/************************************************************************
Queue Delete
************************************************************************/
gpOS_error_t gpOS_message_delete_queue( gpOS_message_queue_t *msg_queue)
{
  gpOS_partition_t *origin_part;
  gpOS_error_t rc;

  if(NULL == msg_queue) {
    rc = gpOS_FAILURE;
  } else {
    // Make sure OS uses in correct partition
    origin_part = memgt_mallopt_set(msg_queue->part);

    vSemaphoreDelete(msg_queue->cnt_sem);
    vQueueDelete(msg_queue->fRtos_queue);

    // Restore Original partion
    memgt_mallopt_restore(origin_part);

    gpOS_memory_deallocate_p(msg_queue->part, msg_queue);
    rc = gpOS_SUCCESS;
  }

  return rc;
}

/************************************************************************
Claim
************************************************************************/
void* gpOS_message_claim( gpOS_message_queue_t *msg_queue)
{
  return gpOS_message_claim_timeout(msg_queue, gpOS_TIMEOUT_INFINITY);
}

/************************************************************************
Claim with TimeOut Management
************************************************************************/
void *gpOS_message_claim_timeout( gpOS_message_queue_t *msg_queue, const gpOS_clock_t *timeout)
{
  FRtoOS_free_msg_block_t *msg_block;
  BaseType_t success;

  // Make sure a block is available
  if(interrupt_nested_count) {
    taskENTER_CRITICAL();
    success = xSemaphoreTakeFromISR(msg_queue->cnt_sem, &xReschedule_fromISR_required);
    taskEXIT_CRITICAL();
  }
  else {
    success = xSemaphoreTake(msg_queue->cnt_sem, OStoFR_timeout(timeout));
  }

  if(pdFALSE == success) {
    msg_block = NULL;
  } else {
    // Grab available message block in list
    taskENTER_CRITICAL();
    msg_block = msg_queue->free_blocks;
    msg_queue->free_blocks = msg_block->next_free;

    // Make sure caller will not pollute us by reusing our pointer
    msg_block->next_free = (FRtoOS_free_msg_block_t *)0xDEADBEAF;
    taskEXIT_CRITICAL();
  }

  return (void *)msg_block;
}

/************************************************************************
Send
************************************************************************/
gpOS_ISR void gpOS_message_send( gpOS_message_queue_t *msg_queue, void *message)
{
  if(interrupt_nested_count) {
    taskENTER_CRITICAL();
    O2F_W_ASSERT(xQueueSendFromISR(msg_queue->fRtos_queue, (void *)&message, &xReschedule_fromISR_required));
    taskEXIT_CRITICAL();
  } else {
    O2F_W_ASSERT(xQueueSend(msg_queue->fRtos_queue, (void *)&message, portMAX_DELAY));
  }
}

/************************************************************************
Receive
************************************************************************/
void *gpOS_message_receive( gpOS_message_queue_t *msg_queue)
{
  return gpOS_message_receive_timeout(msg_queue, gpOS_TIMEOUT_INFINITY);
}

/************************************************************************
Receive with Timeout (FreeRTOS tick resolution)
************************************************************************/
void *gpOS_message_receive_timeout( gpOS_message_queue_t *msg_queue, const gpOS_clock_t *timeout)
{
  int done = 0;
  void *message_ptr;
  BaseType_t receive_rc;
  TickType_t freeRTOSTimeout;

  // Compute target date
  freeRTOSTimeout = (interrupt_nested_count) ?
                    0 : OStoFR_timeout(timeout);

  while(!done) {
    if(interrupt_nested_count) {
      taskENTER_CRITICAL();
      receive_rc = xQueueReceiveFromISR(msg_queue->fRtos_queue, (void *)&message_ptr, &xReschedule_fromISR_required);
      taskEXIT_CRITICAL();
    } else {
      receive_rc = xQueueReceive(msg_queue->fRtos_queue, (void *)&message_ptr, freeRTOSTimeout);
    }

    // User may have previously use HiResolution timeout on this queue
      // When an old time out was in the queue. Ignore it.
      // Assume that it was receive with no delay because already in the queue
    if(    (pdFALSE == receive_rc)
        || (REGULAR_MSG_IDENTIFIER == (((unsigned long)message_ptr) & MESSAGE_IDENTIFIER_MASK))) {
      done = 1;
    }
  }

  if(pdFALSE == receive_rc) {
    message_ptr = NULL;
  }

  return message_ptr;
}

/************************************************************************
Receive with Timeout (Hi Resolution)
This function will internaly schedule a delayed message indicatin timeout
has occured.
************************************************************************/
void *gpOS_message_receive_HiResTimeout( gpOS_message_queue_t *msg_queue, const gpOS_clock_t *timeout)
{
  gpOS_clock_t HiResTimeout;
  svc_job_t timedMsg;
  void *result = NULL;
  int done = 0;
  unsigned long FreeRTOSmessage;
  TickType_t freeRTOS_delay;

  if((gpOS_TIMEOUT_INFINITY == timeout) || (gpOS_TIMEOUT_IMMEDIATE == timeout)) {
    result = gpOS_message_receive_timeout(msg_queue, timeout);
  } else {
    HiResTimeout = (gpOS_clock_t)*timeout;

    // Change Timeout correlator to bypass old message possibly in the queue
    msg_queue->TimeOutMessage += (1 << MESSAGE_IDENTIFIER_MASK_BITS ); // 2 low order bits are unchanged=TIME_OUT_IDENTIFIER

    timer_svc_create(&timedMsg, TIMER_SVC_QUEUE_DELAY,
                      (void *)msg_queue->fRtos_queue,
                      HiResTimeout,
                      (void *)&(msg_queue->TimeOutMessage));
    if(pdTRUE == timer_svc_insert(&timedMsg)) {
      freeRTOS_delay = portMAX_DELAY;
    } else {
      // HiRes delay is too short, do not wait
      freeRTOS_delay = 0;
    }

    while(!done) {
      if ( pdFALSE == xQueueReceive(msg_queue->fRtos_queue, &FreeRTOSmessage, freeRTOS_delay) )
      {
        done = 1;
      }
      else
      {
        switch( FreeRTOSmessage & MESSAGE_IDENTIFIER_MASK) {
          case TIME_OUT_IDENTIFIER :
            if(msg_queue->TimeOutMessage == FreeRTOSmessage) {
              // Time out Received
              done = 1;
          	}
          	break;
        	case REGULAR_MSG_IDENTIFIER:
          	// Cancel the delayed job if not yet expired
          	timer_svc_remove(&timedMsg);
          	result = (void *)FreeRTOSmessage;
          	done = 1;
          	break;
        	default :
          	// Ignore this message
          	break;
         }
      }
    }
  }

  return result;
}

/************************************************************************
Release
************************************************************************/
void gpOS_message_release( gpOS_message_queue_t *msg_queue, void *message)
{
  FRtoOS_free_msg_block_t *new_avail = message;

  // Signal available
  if(interrupt_nested_count) {
    taskENTER_CRITICAL();
    // Put released message block in available list
    new_avail->next_free = msg_queue->free_blocks;
    msg_queue->free_blocks = new_avail;

    O2F_W_ASSERT(xSemaphoreGiveFromISR(msg_queue->cnt_sem, &xReschedule_fromISR_required));
    taskEXIT_CRITICAL();
  } else {
    // Put released message block in available list
    taskENTER_CRITICAL();
    new_avail->next_free = msg_queue->free_blocks;
    msg_queue->free_blocks = new_avail;
    taskEXIT_CRITICAL();

    O2F_W_ASSERT(xSemaphoreGive(msg_queue->cnt_sem));
  }
}
