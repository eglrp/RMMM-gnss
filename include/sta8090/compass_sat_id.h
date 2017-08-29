/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995-2012
                   All Rights Reserved

Source file name : compass_sat_id.h
Description:       Compass Library

Date              Modification
27  Feb 12        Created

************************************************************************/

#ifndef COMPASS_SAT_ID_H
#define COMPASS_SAT_ID_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define COMPASS_MIN_PRN                     1
#define COMPASS_MAX_PRN                     30  // default 37
#define COMPASS_NUM_OF_PRNS                 ((COMPASS_MAX_PRN - COMPASS_MIN_PRN) + 1)

#define MIN_COMPASS_SAT_ID                  141
#define MAX_COMPASS_SAT_ID                  ((MIN_COMPASS_SAT_ID + COMPASS_NUM_OF_PRNS) -1)

#define COMPASS_SAT_ID_VALID(sat_id)        ((((sat_id) >= (satid_t)MIN_COMPASS_SAT_ID)    && ((sat_id) <= (satid_t)MAX_COMPASS_SAT_ID)) ? TRUE:FALSE)

#define NUM_COMPASS_GEO_SAT_IDS             5
#define MIN_COMPASS_GEO_SAT_ID              (MIN_COMPASS_SAT_ID)
#define MAX_COMPASS_GEO_SAT_ID              ((MIN_COMPASS_SAT_ID + NUM_COMPASS_GEO_SAT_IDS) -1)
#define MIN_COMPASS_MEO_SAT_ID              (MAX_COMPASS_GEO_SAT_ID+1)
#define MAX_COMPASS_MEO_SAT_ID              (MAX_COMPASS_SAT_ID)

#define COMPASS_MEO_SAT_ID_VALID(sat_id)    ((((sat_id) >= (satid_t)MIN_COMPASS_MEO_SAT_ID)    && ((sat_id) <= (satid_t)MAX_COMPASS_MEO_SAT_ID)) ? TRUE:FALSE)
#define COMPASS_GEO_SAT_ID_VALID(sat_id)    ((((sat_id) >= (satid_t)MIN_COMPASS_GEO_SAT_ID)    && ((sat_id) <= (satid_t)MAX_COMPASS_GEO_SAT_ID)) ? TRUE:FALSE)

#define NUM_COMPASS_IGSO_SAT_IDS                5
#define MAX_COMPASS_GEO_IGSO_SAT_ID             ((MIN_COMPASS_SAT_ID + NUM_COMPASS_GEO_SAT_IDS + NUM_COMPASS_IGSO_SAT_IDS) -1)
#define COMPASS_GEO_IGSO_SAT_ID_VALID(sat_id)   ((((sat_id) >= (satid_t)MIN_COMPASS_GEO_SAT_ID)    && ((sat_id) <= (satid_t)MAX_COMPASS_GEO_IGSO_SAT_ID)) ? TRUE:FALSE)

#define COMPASS_SAT_ID_TO_PRN(sat_id)       (((sat_id) - MIN_COMPASS_SAT_ID) + 1)
#define COMPASS_PRN_TO_SAT_ID(prn)          (((prn) + MIN_COMPASS_SAT_ID) - 1)

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* COMPASS_SAT_ID_H */
