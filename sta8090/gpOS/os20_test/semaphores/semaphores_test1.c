/*****************************************************************************
   FILE:          semaphores_test1.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Semaphores
                  (OS20+ operative system specification, Chapter 4)
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics,
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
#include "gpOS_task.h"
#include "gnss_debug.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define SEMAPHORES_TEST1_WS_SIZE      512

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/


/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static gpOS_task_t *semaphores_test1_task1;
static gpOS_task_t *semaphores_test1_task2;
static gpOS_task_t *semaphores_test1_task3;
static gpOS_task_t *semaphores_test1_task4;
static gpOS_task_t *semaphores_test1_task5;
static gpOS_task_t *semaphores_test1_task6;
static gpOS_task_t *semaphores_test1_task7;
static gpOS_task_t *semaphores_test1_task8;

static gpOS_semaphore_t *semaphores_test1_sem1;
static gpOS_semaphore_t *semaphores_test1_sem2;
static gpOS_semaphore_t *semaphores_test1_sem3;
static gpOS_semaphore_t *semaphores_test1_sem4;
static gpOS_semaphore_t *semaphores_test1_sem5;
static gpOS_semaphore_t *semaphores_test1_sem6;
static gpOS_semaphore_t *semaphores_test1_sem7;
static gpOS_semaphore_t *semaphores_test1_sem8;

static gpOS_clock_t time_table_1[4];
static gpOS_clock_t time_table_2[4];
static gpOS_clock_t time_table_3[4];
static gpOS_clock_t time_table_4[4];

static gpOS_clock_t time_table_5[4];
static gpOS_clock_t time_table_6[4];
static gpOS_clock_t time_table_7[4];
static gpOS_clock_t time_table_8[4];


unsigned counter1 = 0, counter2 = 0, counter3 = 0, counter4 = 0, counter5 = 0, counter6 = 0, counter7 = 0, counter8 = 0;

static gpOS_task_exit_status_t semaphores_test1_running1( void *dummy)
{

  while(1)
  {

    gpOS_semaphore_wait( semaphores_test1_sem1);

    time_table_1[counter1] = gpOS_time_now();

    counter1++;

  }
}

static gpOS_task_exit_status_t semaphores_test1_running2( void *dummy)
{

  while(1)
  {

    gpOS_semaphore_wait( semaphores_test1_sem2);

    time_table_2[counter2] = gpOS_time_now();

    counter2++;

  }
}

static gpOS_task_exit_status_t semaphores_test1_running3( void *dummy)
{

  while(1)
  {

    gpOS_semaphore_wait( semaphores_test1_sem3);

    time_table_3[counter3] = gpOS_time_now();

    counter3++;

  }
}

static gpOS_task_exit_status_t semaphores_test1_running4( void *dummy)
{

  while(1)
  {

    gpOS_semaphore_wait( semaphores_test1_sem4);

    time_table_4[counter4] = gpOS_time_now();

    counter4++;

  }
}

static gpOS_task_exit_status_t semaphores_test1_running5( void *dummy)
{

  while(1)
  {

    gpOS_semaphore_wait( semaphores_test1_sem5);

    time_table_5[counter5] = gpOS_time_now();

    counter5++;

  }
}

static gpOS_task_exit_status_t semaphores_test1_running6( void *dummy)
{

  while(1)
  {

    gpOS_semaphore_wait( semaphores_test1_sem6);

    time_table_6[counter6] = gpOS_time_now();

    counter6++;

  }
}

static gpOS_task_exit_status_t semaphores_test1_running7( void *dummy)
{

  while(1)
  {

    gpOS_semaphore_wait( semaphores_test1_sem7);

    time_table_7[counter7] = gpOS_time_now();

    counter7++;

  }
}

static gpOS_task_exit_status_t semaphores_test1_running8( void *dummy)
{

  while(1)
  {

    gpOS_semaphore_wait( semaphores_test1_sem8);

    time_table_8[counter8] = gpOS_time_now();

    counter8++;

  }
}

gpOS_error_t semaphores_test1_run( void)
{
  unsigned counter = 0; //, runs = 0;

  if(( semaphores_test1_task1 == NULL) || ( semaphores_test1_task2 == NULL) ||
     ( semaphores_test1_task3 == NULL) || ( semaphores_test1_task4 == NULL) ||
     ( semaphores_test1_task5 == NULL) || ( semaphores_test1_task6 == NULL) ||
     ( semaphores_test1_task7 == NULL) || ( semaphores_test1_task8 == NULL))
  {
    semaphores_test1_task1 = gpOS_task_create( semaphores_test1_running1, NULL, SEMAPHORES_TEST1_WS_SIZE, 10, "Semaphore T1 P1", gpOS_TASK_FLAGS_ACTIVE);
    semaphores_test1_task2 = gpOS_task_create( semaphores_test1_running2, NULL, SEMAPHORES_TEST1_WS_SIZE, 10, "Semaphore T1 P2", gpOS_TASK_FLAGS_ACTIVE);
    semaphores_test1_task3 = gpOS_task_create( semaphores_test1_running3, NULL, SEMAPHORES_TEST1_WS_SIZE, 10, "Semaphore T1 P3", gpOS_TASK_FLAGS_ACTIVE);
    semaphores_test1_task4 = gpOS_task_create( semaphores_test1_running4, NULL, SEMAPHORES_TEST1_WS_SIZE, 10, "Semaphore T1 P4", gpOS_TASK_FLAGS_ACTIVE);
    semaphores_test1_task5 = gpOS_task_create( semaphores_test1_running5, NULL, SEMAPHORES_TEST1_WS_SIZE, 10, "Semaphore T1 P5", gpOS_TASK_FLAGS_ACTIVE);
    semaphores_test1_task6 = gpOS_task_create( semaphores_test1_running6, NULL, SEMAPHORES_TEST1_WS_SIZE, 10, "Semaphore T1 P6", gpOS_TASK_FLAGS_ACTIVE);
    semaphores_test1_task7 = gpOS_task_create( semaphores_test1_running7, NULL, SEMAPHORES_TEST1_WS_SIZE, 10, "Semaphore T1 P7", gpOS_TASK_FLAGS_ACTIVE);
    semaphores_test1_task8 = gpOS_task_create( semaphores_test1_running8, NULL, SEMAPHORES_TEST1_WS_SIZE, 10, "Semaphore T1 P8", gpOS_TASK_FLAGS_ACTIVE);


    semaphores_test1_sem1 = gpOS_semaphore_create(SEM_FIFO, 0);
    semaphores_test1_sem2 = gpOS_semaphore_create(SEM_FIFO, 1);
    semaphores_test1_sem3 = gpOS_semaphore_create(SEM_FIFO, 2);
    semaphores_test1_sem4 = gpOS_semaphore_create(SEM_FIFO, 3);

    semaphores_test1_sem5 = gpOS_semaphore_create(SEM_FIFO, 0);
    semaphores_test1_sem6 = gpOS_semaphore_create(SEM_FIFO, 0);
    semaphores_test1_sem7 = gpOS_semaphore_create(SEM_FIFO, 0);
    semaphores_test1_sem8 = gpOS_semaphore_create(SEM_FIFO, 0);

    if(( semaphores_test1_task1 == NULL) || ( semaphores_test1_task2 == NULL) ||
       ( semaphores_test1_task3 == NULL) || ( semaphores_test1_task4 == NULL) ||
       ( semaphores_test1_task5 == NULL) || ( semaphores_test1_task6 == NULL) ||
       ( semaphores_test1_task7 == NULL) || ( semaphores_test1_task8 == NULL) ||
       ( semaphores_test1_sem1  == NULL) || ( semaphores_test1_sem2  == NULL) ||
       ( semaphores_test1_sem3  == NULL) || ( semaphores_test1_sem4  == NULL) ||
       ( semaphores_test1_sem5  == NULL) || ( semaphores_test1_sem6  == NULL) ||
       ( semaphores_test1_sem7  == NULL) || ( semaphores_test1_sem8  == NULL)
      )
    {
      return gpOS_FAILURE;
    }
  }

  counter = counter1 = counter2 = counter3 = counter4 = counter5 = counter6 = 0;

  gpOS_semaphore_signal( semaphores_test1_sem1);
  gpOS_semaphore_signal( semaphores_test1_sem2);
  gpOS_semaphore_signal( semaphores_test1_sem3);
  gpOS_semaphore_signal( semaphores_test1_sem4);

  gpOS_semaphore_signal( semaphores_test1_sem5);
  gpOS_semaphore_signal( semaphores_test1_sem6);
  gpOS_semaphore_signal( semaphores_test1_sem7);
  gpOS_semaphore_signal( semaphores_test1_sem8);

  GPS_DEBUG_MSG(( "\r\n[Semaphores] [1] [Task#] [Counter] [t1] [t2] [t3] [t4]\r\n\r\n"));

  while( counter < 10)
  {

    gpOS_task_delay( gpOS_timer_ticks_per_sec());
    GPS_DEBUG_MSG(( "." ));
    counter++;

  }

  GPS_DEBUG_MSG(( " done\r\n" ));

  GPS_DEBUG_MSG(( "[Semaphores] [1] [Task1] [%d] [%d] [%d] [%d] [%d]\r\n",
                   counter1,
                   time_table_1[0],
                   time_table_1[1],
                   time_table_1[2],
                   time_table_1[3]));

  GPS_DEBUG_MSG(( "[Semaphores] [1] [Task2] [%d] [%d] [%d] [%d] [%d]\r\n",
                   counter2,
                   time_table_2[0],
                   time_table_2[1],
                   time_table_2[2],
                   time_table_2[3]));

  GPS_DEBUG_MSG(( "[Semaphores] [1] [Task3] [%d] [%d] [%d] [%d] [%d]\r\n",
                   counter3,
                   time_table_3[0],
                   time_table_3[1],
                   time_table_3[2],
                   time_table_3[3]));

  GPS_DEBUG_MSG(( "[Semaphores] [1] [Task4] [%d] [%d] [%d] [%d] [%d]\r\n",
                   counter4,
                   time_table_4[0],
                   time_table_4[1],
                   time_table_4[2],
                   time_table_4[3]));

  GPS_DEBUG_MSG(( "[Semaphores] [1] [Task5] [%d] [%d] [%d] [%d] [%d]\r\n",
                   counter5,
                   time_table_5[0],
                   time_table_5[1],
                   time_table_5[2],
                   time_table_5[3]));

  GPS_DEBUG_MSG(( "[Semaphores] [1] [Task6] [%d] [%d] [%d] [%d] [%d]\r\n",
                   counter6,
                   time_table_6[0],
                   time_table_6[1],
                   time_table_6[2],
                   time_table_6[3]));

  GPS_DEBUG_MSG(( "[Semaphores] [1] [Task7] [%d] [%d] [%d] [%d] [%d]\r\n",
                   counter7,
                   time_table_7[0],
                   time_table_7[1],
                   time_table_7[2],
                   time_table_7[3]));

  GPS_DEBUG_MSG(( "[Semaphores] [1] [Task8] [%d] [%d] [%d] [%d] [%d]\r\n",
                   counter8,
                   time_table_8[0],
                   time_table_8[1],
                   time_table_8[2],
                   time_table_8[3]));

  GPS_DEBUG_MSG(( "\r\n" ));

  return gpOS_SUCCESS;
}
/*}}}  */
