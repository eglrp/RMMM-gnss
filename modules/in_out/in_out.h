//!
//!   \file     in_out.h
//!   \brief    <i><b> Input/output management header file</b></i>
//!   \author   Giovanni De Angelis
//!   \version  1.0
//!   \date     2012.02.15
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef INOUT_H
#define INOUT_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "nmea.h"
#include "gnss_debug.h"

// RTCM related
#include "dgps.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define LLD_SQI_SECTOR_SIZE           (64 * 1024)
#define LLD_SQI_SUBSECTOR_SIZE        (4 * 1024)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  IN_OUT_DEBUG_PORT = 0x00,
  IN_OUT_PORT_COUNT,
} in_out_port_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t     in_out_start          ( gpOS_partition_t *part);

extern gpOS_error_t     in_out_get_debug_cfg  ( gnss_debug_inout_t *debug_input_func, gnss_debug_writeout_t *debug_output_func);
extern gpOS_error_t     in_out_get_nmea_cfg   ( nmea_inout_t *nmea_input_func, nmea_inout_t *nmea_output_func);
extern gpOS_error_t     in_out_get_rtcm_cfg   ( dgps_read_t *dgps_input_func);

extern gpOS_error_t     in_out_open_port      ( in_out_port_t port);
extern gpOS_error_t     in_out_close_port     ( in_out_port_t port);

extern void             in_out_update         ( void);

#endif  // INOUT_H

// End of file
