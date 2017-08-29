/**
 * @file    task.h
 * @brief   OS tasks definitions and macros.
 *
 * @addtogroup OS
 */

#ifndef gpOS_TASK_H
#define gpOS_TASK_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS_time.h"
#include "gpOS_memory.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define gpOS_TASK_PRIORITY_LEVELS     16                                  /**< OS priorities */
#define gpOS_TASK_MAX_USR_PRIORITY    (gpOS_TASK_PRIORITY_LEVELS - 1)     /**< max OS priority */
#define gpOS_TASK_MIN_USR_PRIORITY    0                                   /**< min OS priority */

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

//
// task state type
//
typedef enum gpOS_task_state_e
{
  gpOS_TASK_STATE_ACTIVE      = 0,
  gpOS_TASK_STATE_TERMINATED  = 1,
  gpOS_TASK_STATE_SUSPENDING  = 2,
  gpOS_TASK_STATE_SUSPENDED   = 3
} gpOS_task_state_t;                                  /**< state of a task */

//
// task flags type
//
typedef enum gpOS_task_flags_e
{
  gpOS_TASK_FLAGS_ACTIVE     = 0,
  gpOS_TASK_FLAGS_SUSPENDED  = 1
} gpOS_task_flags_t;                                  /**< current status of a task */

//
// types for task structure
//
typedef tInt                gpOS_task_exit_status_t;  /**< exit status of a task procedure */
typedef void *              gpOS_task_param_t;        /**< parameter type for a task procedure */
typedef struct gpOS_task_s  gpOS_task_t;              /**< task type */

typedef gpOS_task_exit_status_t (*gpOS_task_function_t) ( gpOS_task_param_t);  /**< type of procedure that will be executed by task */

//
// type for function used when task exits
//
typedef void (*gpOS_task_onexit_function_t) ( gpOS_task_t *, tInt);   /**< function called exiting a task */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_task_t*   gpOS_task_create               ( gpOS_task_function_t proc_func, gpOS_task_param_t proc_param, const tSize stack_size, const tInt priority, const tChar *name, gpOS_task_flags_t flags);
extern gpOS_task_t*   gpOS_task_create_p             ( gpOS_partition_t* custom_part, gpOS_task_function_t proc_func, gpOS_task_param_t proc_param, const tSize stack_size, const tInt priority, const tChar *name, gpOS_task_flags_t flags);

extern gpOS_error_t   gpOS_task_delete               ( gpOS_task_t* task);

extern void           gpOS_task_delay                ( const gpOS_clock_t delay);
extern void           gpOS_task_delay_until          ( const gpOS_clock_t timeout);
extern gpOS_error_t   gpOS_task_suspend              ( gpOS_task_t* task);
extern gpOS_error_t   gpOS_task_resume               ( gpOS_task_t* task);

extern gpOS_task_t*   gpOS_task_get_head             ( void);
extern gpOS_task_t*   gpOS_task_get_next             ( gpOS_task_t* task);
extern gpOS_task_t *  gpOS_task_get_id               ( void);

extern tInt           gpOS_task_set_priority         ( gpOS_task_t* task, const tInt new_priority);
extern tInt           gpOS_task_get_priority         ( gpOS_task_t* task);
extern void*          gpOS_task_get_stack_base       ( gpOS_task_t* task);
extern void*          gpOS_task_get_stack_ptr        ( gpOS_task_t* task);
extern tSize          gpOS_task_get_stack_size       ( gpOS_task_t* task);
extern tSize          gpOS_task_get_stack_used       ( gpOS_task_t* task);
extern tUInt          gpOS_task_get_cpuusage         ( gpOS_task_t* task);
extern tUInt          gpOS_task_get_runtime          ( gpOS_task_t* task);
extern const tChar*   gpOS_task_get_name             ( gpOS_task_t* task);
extern void*          gpOS_task_get_data             ( gpOS_task_t* task);
extern void*          gpOS_task_set_data             ( gpOS_task_t* task, void* data);
extern gpOS_clock_t   gpOS_task_get_timeout          ( gpOS_task_t* task);

extern tInt           gpOS_task_wait                 ( gpOS_task_t** gpOS_task_list, tInt ntasks, const gpOS_clock_t* timeout);

extern void           gpOS_task_lock                 ( void);
extern void           gpOS_task_unlock               ( void);

extern void                         gpOS_task_exit                ( gpOS_task_exit_status_t exit_status);
extern gpOS_task_onexit_function_t  gpOS_task_set_onexit_function ( gpOS_task_onexit_function_t function);

#endif /* TASK_H */
