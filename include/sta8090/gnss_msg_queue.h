/*****************************************************************************
   FILE:          <filename>
   PROJECT:       <project name>
   SW PACKAGE:    <software package name>
------------------------------------------------------------------------------
   DESCRIPTION:   <...>
------------------------------------------------------------------------------
   COPYRIGHT:     (c) <year> STMicroelectronics, (<group>) <site>
------------------------------------------------------------------------------
   Developers:
      AI:   Author Initials
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
   ----------+------+------------------------------------------------------
   yy.mm.dd  |  AI  | Original version
*****************************************************************************/
#ifndef GNSS_MSG_QUEUE_H
#define GNSS_MSG_QUEUE_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define MSG_QUEUE_LEN     (8192)

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef enum {
  MSG_SENSORS_ID = 0,
  MSG_SENSORS_MEMS_ID,
  MSG_SENSORS_SPEED_ODO_ID,
  MSG_NAV_CMD_ID,
  MSG_ANALOG_ID,
  MSG_3DACC_ID,
  MSG_3DGYRO_ID,
  MSG_PRES_ID,
  MSG_CAN_ID,
  MSG_DEBUG_ID,
  MSG_APP_SENSORS_ID
} gnss_msg_queue_ids_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

extern tU8* gnss_msg_queue;

extern gnss_msg_reader_handler_t nmea_analog_amq_reader;
extern gnss_msg_reader_handler_t nmea_3dacc_amq_reader;
extern gnss_msg_reader_handler_t nmea_3dgyro_amq_reader;
extern gnss_msg_reader_handler_t nmea_can_amq_reader;
extern gnss_msg_reader_handler_t nmea_pres_amq_reader;
extern gnss_msg_reader_handler_t nmea_debug_amq_reader;
extern gnss_msg_reader_handler_t nav_dr_amq_reader;
extern gnss_msg_reader_handler_t app_amq_reader;
extern gnss_msg_reader_handler_t nmea_sens_amq_reader;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gnss_error_t amq_start                         ( void);
extern gnss_error_t amq_start_with_dr                 ( void);
extern gnss_error_t app_init_register_AMQ_reader      ( gnss_msg_reader_handler_t** reader_hnd_ptr, const gnss_msg_id_t) ;
extern gnss_error_t gnss_msg_queue_Nmea_register      ( void);
extern gnss_error_t gnss_msg_queue_dr_plugin_register ( tU8, const gnss_msg_id_t *);

#endif
