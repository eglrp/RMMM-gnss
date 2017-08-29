/*****************************************************************************
   FILE:          st_agps.h
   PROJECT:       GPS library
   SW PACKAGE:    ST Assisted GPS
------------------------------------------------------------------------------
   DESCRIPTION:   This is the header for the ST assisted GPS module.
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2008 STMicroelectronics
------------------------------------------------------------------------------
   Developers:
      FB:   Fulvio Boggia
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
   ----------+------+------------------------------------------------------
   08.01.21  |  FB  | Original version
*****************************************************************************/

#ifndef ST_AGPS_H
#define ST_AGPS_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"
#include "gnss_defs.h"
#include "gnss_os.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define ST_AGPS_CLR_EPHDB         (1<<0)
#define ST_AGPS_CLR_SAGPS_SEEDDB  (1<<1)
#define ST_AGPS_CLR_SAGPS_POLYDB  (1<<2)
#define ST_AGPS_CLR_PGPS_SEED     (1<<3)
#define ST_AGPS_CLR_PGPS_SEEDDB   (1<<4)
#define ST_AGPS_CLR_PGPS_POLYDB   (1<<5)
#define ST_AGPS_CLR_ALL           0xff

#define STAGPS_PGPS_LINKED_SAT_IDS        (NUM_GPS_SAT_IDS + NUM_GLONASS_SAT_IDS)

#define STAGNSS_DEFAULT_CONSTELLATION_MASK (STAGNSS_GPS_CONSTELLATION)

/*****************************************************************************
   typedefs and structures (scope: exported)
*****************************************************************************/

typedef enum stagps_constellation_e
{
    STAGPS_GPS_ONLY,
    STAGPS_GPS_GLO
} stagps_constellation_t;

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum ST_AGPS_suspend_s {
  ST_AGPS_SUSPEND_IMMEDIATELY,
  ST_AGPS_SUSPEND_ONIDLE
} ST_AGPS_suspend_t;

typedef enum ST_AGPS_mgrid_s {
  ST_AGPS_MGRID_EPH,
  ST_AGPS_MGRID_SAGPS,
  ST_AGPS_MGRID_PGPS
} ST_AGPS_mgrid_t;

typedef tU8 ST_AGPS_clr_t;

/*****************************************************************************
   exported variables
*****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

extern const tS8 *stagpslib_ver;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gnss_error_t   ST_AGPS_init                      ( gpOS_partition_t *);
extern void           ST_AGPS_start                     ( void);
extern void           ST_AGPS_start_initial_prediction  ( void);
extern void           ST_AGPS_start_pgps                ( tBool);
extern void           ST_AGPS_suspend                   ( const ST_AGPS_suspend_t);
extern tBool          ST_AGPS_suspended                 ( void);
extern const tS8 *    ST_AGPS_version                   ( void);

extern tS32           ST_AGPS_get_max_sat_ids           ( void);
extern tS32           ST_AGPS_get_min_sat_ids           ( void);
extern gnss_error_t   ST_AGPS_set_constellation         ( stagps_constellation_t constellation);
extern gnss_error_t   ST_AGPS_set_constellation_mask    ( tU8 mask);
extern void           ST_AGPS_invalidate                ( const satid_t, const ST_AGPS_clr_t);
extern gpOS_task_t *  ST_AGPS_get_task_ptr              ( const ST_AGPS_mgrid_t);

extern tBool          ST_AGPS_load_single_pgps          ( gnsslibid_t idx, tU8* seed_buf, tU32 seed_t0, tU32 seed_tcur, tS32 seed_tau_gps, tS32 seed_tau_gps_dot, tU8 gps_utc_offset, tBool force);

#ifdef __cplusplus
}
#endif
#endif /* _ST_AGPS_H */
