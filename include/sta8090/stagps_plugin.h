/*****************************************************************************
   FILE:          stagps_plugin.h
   PROJECT:       STA2062 GPS application
   SW PACKAGE:    STA2062 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   The main application to run and test STA2062 GPS library
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2014 STMicroelectronics
------------------------------------------------------------------------------
   Created by : Michel Susini
           on : W421
*****************************************************************************/

#ifndef STAGPS_PLUGINS
#define STAGPS_PLUGINS

/*****************************************************************************
   includes
*****************************************************************************/
#include "nmea.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define PGPS_PWDGEN_MAX_FIELD_LENGTH  50
#define PASSWORD_LENGTH               41
  #define STAGPS_NMEA_PGPSSEEDMAXSIZE   400
#define STAGPS_MAX_LIST_LEN           (NMEA_GSV_MAX_SATS * 2)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef struct
{
  tU8 sat_idx_counter;
  tU8 sat_idx_pgps_max;
} stagps_plugin_handler_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* STAGPS_PLUGINS */

