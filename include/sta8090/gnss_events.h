/*{{{  COMMENT Standard Header*/
/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995,1996,1997,1998,1999
                   All Rights Reserved

Source file name : gnss_events.h
Author :           A. Di Girolamo

GNSS Events Management

Date        Modification                                    Initials
----        ------------                                    --------
03 Dec 10   Created                                           ADG

************************************************************************/
/*}}}  */

#ifndef GNSS_EVENTS_H
#define GNSS_EVENTS_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define GNSS_EVENTS_SYNCHANDLER_NONE        ((gnss_events_synch_handler_t *)NULL)

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef enum
{
  GNSS_EVENTID_FIXREADY,
  GNSS_EVENTID_NVMENTER,
  GNSS_EVENTID_NVMCHANGEDREGION,
  GNSS_EVENTID_NVMEXIT,
  GNSS_EVENTID_PPS,
  GNSS_EVENTID_DRFIXREADY,
  GNSS_EVENTID_NUMBER
} gnss_events_event_id_t;

typedef void* gnss_events_callback_param_t;

typedef void (*gnss_events_callback_t)(gnss_events_callback_param_t);

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

typedef struct gnss_events_synch_handler_s
{
  struct gnss_events_synch_handler_s *  next_synch_handler;
  gpOS_semaphore_t *                    synch_semaphore;
} gnss_events_synch_handler_t;

#ifdef __cplusplus
extern "C" {
#endif

extern gpOS_error_t                   gnss_events_init                    ( void);
extern gpOS_error_t                   gnss_events_init_p                  ( gpOS_partition_t *);
extern gnss_events_synch_handler_t *  gnss_events_synch_handler_create    ( void);
extern void                           gnss_events_synch_handler_delete    ( gnss_events_synch_handler_t *);
extern void                           gnss_events_install                 ( gnss_events_event_id_t, gnss_events_synch_handler_t *);
extern void                           gnss_events_uninstall               ( gnss_events_event_id_t, gnss_events_synch_handler_t *);
extern void                           gnss_events_set_callback            ( gnss_events_event_id_t, gnss_events_callback_t) ;
extern void                           gnss_events_signal                  ( gnss_events_event_id_t, gnss_events_callback_param_t);
extern void                           gnss_events_wait                    ( gnss_events_event_id_t, gnss_events_synch_handler_t *);
extern gpOS_error_t                   gnss_events_wait_timeout            ( gnss_events_event_id_t, gnss_events_synch_handler_t *, gpOS_clock_t *);
#ifdef __cplusplus
}
#endif
#endif /* GNSS_EVENTS_H */
