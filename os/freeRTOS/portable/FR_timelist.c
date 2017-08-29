/*{{{  title*/
/*
 * freeRTOS_timelist.c
 *
 * Copyright (C) STMicroelectronics Ltd. 2001
 *
 * Timelist handling.
 */
/*}}}  */

/*****************************************************************************
   includes
*****************************************************************************/

/*{{{  include*/
#include "gpOS_time.h"
#include "gpOS_timelist.h"
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

/*{{{  gpOS_timer_list_init()*/
void gpOS_timer_list_init( gpOS_clock_t *timer_list, const int num_timers)
{
  int i;
  gpOS_clock_t t_now;

  t_now = gpOS_time_now();

  for(i = 0; i < num_timers; i++)
  {
    timer_list[i] = t_now;
  }
}
/*}}}  */
/*{{{  gpOS_timer_list_update()*/

void gpOS_timer_list_update(gpOS_clock_t *timer_list, const int timer_index, const int timer_interval)
{
  timer_list[timer_index] = gpOS_time_plus( gpOS_time_now(), timer_interval);

  /*timer_list[timer_index] = gpOS_time_plus(timer_list[timer_index], timer_interval);*/
}
/*}}}  */
/*{{{  gpOS_timer_list_calc_nearest()*/
void gpOS_timer_list_calc_nearest(gpOS_clock_t *timer_list, const int num_timers, int *nearest, gpOS_clock_t *nearest_time)
{
  int i;

  *nearest_time = timer_list[0];
  *nearest = 0;

  for(i = 1; i < num_timers; i++)
  {
    if(gpOS_time_after(*nearest_time, timer_list[i]))
    {
      *nearest_time = timer_list[i];
      *nearest = i;
    }
  }
}
/*}}}  */
/*{{{  gpOS_timer_list_update_absolute()*/

void gpOS_timer_list_update_absolute(gpOS_clock_t *timer_list, const int timer_index, const gpOS_clock_t timer_value)
{
  timer_list[timer_index] = timer_value;

  /*timer_list[timer_index] = gpOS_time_plus(timer_list[timer_index], timer_interval);*/
}
/*}}}  */
