/*{{{  COMMENT Standard Header*/

/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995,1996,1997,1998,1999,2004,2006
                   All Rights Reserved

Source file name : waas_defs.h
Author :           Dante Di Domenico
Description:       Waas definitions


Date              Modification                            Initials
1   Oct 04        Created                                 DDD
6   Jun 06        New defs for the NVM storing            DDD

************************************************************************/
/*}}}  */

#ifndef WAAS_DEFS_H
#define WAAS_DEFS_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "typedefs.h"
#include "gnss_defs.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

/**********  From Waas  **********/
typedef  struct sSatData
{
  tUInt       fast        : 12;
  tUInt       fast_udrei  :  4;
  tUInt       dx          :  9;
  tUInt       dy          :  9;
  tUInt       dz          :  9;
  tUInt       dafo        : 10;
  tUShort     tow_fast;
  tUShort     tow_slow;
  tUShort     week_n;
  tUChar      iode;
  tUChar      reserved;        // was previously redundant information (prn)
} tSatData;

/******  From Satellite  ******/
typedef tUInt tSatid;

/******  Correction data  ******/
typedef struct sGpsSatCorrection
{
  tUChar      valid;
  tInt        tow_fast;
  tInt        tow_slow;
  tDouble     fast;
  tDouble     slow;
  tDouble     dx;
  tDouble     dy;
  tDouble     dz;
  tDouble     clock;
  //tInt        iode;
} tGpsSatCorrection;

typedef struct sGeoSatParam
{
  tDouble     xdiff;
  tDouble     ydiff;
  tDouble     zdiff;
} tGeoSatParam;

/**** SBAS satellite list managing types for the automatic search *****/
typedef enum
{
  NO_SERVICE  = -1,
  WAAS        = 0,
  EGNOS       = 1,
  MSAS        = 2,
  GAGAN       = 3
} tSbasService;

typedef struct sSbasInfo
{
  tSbasService    service;
  tUChar          status;
  tUInt           prn;
  tDouble         longitude;

} tSbasInfo;

typedef struct sSbasServiceLimit
{
  tInt        min_longitude;
  tInt        max_longitude;

} tServiceLimit;

typedef struct sSbasSatelliteParams
{
  tUInt       msg_id;
  tInt        min_longitude;
  tInt        max_longitude;

} tSbasSatelliteParams;

typedef union
{
  struct sSatelliteParams
  {
    tUInt sat_id      : 8;
    tUInt longitude   : 8;
    tUInt sense       : 1;
    tUInt service     : 2;
    tUInt reserverd   :13;
  } SatParam;
  tUInt param;
} tSatelliteParams;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif
