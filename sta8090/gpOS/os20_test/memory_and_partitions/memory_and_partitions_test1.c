/*****************************************************************************
   FILE:          memory_and_partitions_test1.c
   PROJECT:       STA8090 GNSS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Memory and Partitions
                  (OS20+ operative system specification, Chapter 2)
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2011 STMicroelectronics,
------------------------------------------------------------------------------
   Developers:
      AO:   Aldo Occhipinti
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
 ------------+------+------------------------------------------------------
             |      |
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "gnss_debug.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define TEST1_WS_SIZE            1024
#define NEW_PARTITION_BASE_1     ((void *)((tUInt)0x120000))
#define NEW_PARTITION_BASE_2     ((void *)((tUInt)0x124000))
#define NEW_PARTITION_BASE_3     ((void *)((tUInt)0x128000))
#define BLOCK_SIZE               (4*1024)
#define NEW_PARTITION_SIZE       0x4000  /* 16 kB */

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
typedef struct {

   tU32 field1[256];
   tU32 field2[256];

} BLOCK1;

typedef struct {

   tU32 field1[256];
   tU32 field2[256];

} BLOCK2;


typedef struct {

   tU32 field1[256];
   tU32 field2[256];

} BLOCK3;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
tUInt memory_test1_task_priority = 10;
gpOS_partition_t *part1 = NULL;              /**< partition used for dynamic allocation */
gpOS_partition_t *part2 = NULL;              /**< partition used for dynamic allocation */
gpOS_partition_t *part3 = NULL;              /**< partition used for dynamic allocation */
gpOS_task_t      *task1;                     /**< task pointer */
gpOS_task_t      *task2;                     /**< task pointer */
gpOS_task_t      *task3;                     /**< task pointer */
tChar *buf1, *buf2, *buf3;

/********************************************//**
 * \brief
 *
 * \param dummy task_param_t
 * \return task_exit_status_t
 ***********************************************/
gpOS_ISR gpOS_task_exit_status_t memory_test1_process_1( gpOS_task_param_t dummy)
{

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param dummy task_param_t
 * \return task_exit_status_t
 ***********************************************/
gpOS_ISR gpOS_task_exit_status_t memory_test1_process_2( gpOS_task_param_t dummy)
{

  return gpOS_SUCCESS;

}

/********************************************//**
 * \brief
 *
 * \param dummy task_param_t
 * \return task_exit_status_t
 ***********************************************/
gpOS_ISR gpOS_task_exit_status_t memory_test1_process_3( gpOS_task_param_t dummy)
{
  gpOS_task_t*  task;


  task = gpOS_task_get_head();

  while ( task != NULL )
  {

   GPS_DEBUG_MSG(("[Memory] [1] [%s] [0x%08x]\r\n",
                   gpOS_task_get_name( task ),
                 ( tUInt )gpOS_task_get_stack_base( task )));

   task = gpOS_task_get_next( task );

    }

  return gpOS_SUCCESS;
}


/********************************************//**
 * \brief
 *
 * \param void
 * \return os20_error_t
 ***********************************************/
gpOS_error_t memory_and_partitions_test1_run( gpOS_partition_t *part)
{

  part1 = gpOS_memory_create_partition( gpOS_MEMORY_TYPE_SIMPLE, NEW_PARTITION_BASE_1, NEW_PARTITION_SIZE);
  buf1  = (tChar *)gpOS_memory_allocate_p( part1, sizeof( BLOCK1));

  part2 = gpOS_memory_create_partition( gpOS_MEMORY_TYPE_SIMPLE, NEW_PARTITION_BASE_2, NEW_PARTITION_SIZE);
  buf2  = (tChar *)gpOS_memory_allocate_p( part2, sizeof( BLOCK2));

  part3 = gpOS_memory_create_partition( gpOS_MEMORY_TYPE_SIMPLE, NEW_PARTITION_BASE_3, NEW_PARTITION_SIZE);
  buf3  = (tChar *)gpOS_memory_allocate_p( part3, sizeof( BLOCK3));


  task1 = gpOS_task_create_p ( part1, memory_test1_process_1, NULL, TEST1_WS_SIZE, memory_test1_task_priority, "Memory T1 P1", gpOS_TASK_FLAGS_ACTIVE);

  if( task1 == NULL)
    {

       gpOS_task_delete( task1);
       GPS_DEBUG_MSG(( " task_create() failed 1 \n"));

       return gpOS_FAILURE;

    };

  task2 = gpOS_task_create_p ( part2, memory_test1_process_2, NULL, TEST1_WS_SIZE, memory_test1_task_priority, "Memory T1 P2", gpOS_TASK_FLAGS_ACTIVE);

  if( task2 == NULL)
  {

     gpOS_task_delete( task2);
     GPS_DEBUG_MSG(( " task_create() failed 2 \n"));

     return gpOS_FAILURE;

  };


  task3 = gpOS_task_create_p ( part3, memory_test1_process_3, NULL, TEST1_WS_SIZE, memory_test1_task_priority, "Memory T1 P3", gpOS_TASK_FLAGS_ACTIVE);

  if( task3 == NULL)
  {

     gpOS_task_delete( task3);
     GPS_DEBUG_MSG(( " task_create() failed 3 \n"));

     return gpOS_FAILURE;

  };


  GPS_DEBUG_MSG(( "\r\n[Memory] [1] [TASK NAME] [TASK STACK BASE]\r\n\r\n"));

  return gpOS_SUCCESS;
}
/*}}}  */
