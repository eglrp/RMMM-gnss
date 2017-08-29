/*****************************************************************************
   FILE:          st_agps_testing.h
   PROJECT:       GPS library
   SW PACKAGE:    ST Assisted GPS
------------------------------------------------------------------------------
   DESCRIPTION:   This is the header for the ST assisted GPS module testing
                  features.
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

#ifndef ST_AGPS_TESTING_H
#define ST_AGPS_TESTING_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern void ST_AGPS_TST_invalidate_all_databases(void);
extern tS32 ST_AGPS_TST_get_state(void);

// ephs database related
extern void ST_AGPS_TST_ephdb_claim( void);
extern void ST_AGPS_TST_ephdb_release( void);

// polys database related
extern void ST_AGPS_TST_sagpsmgr_start(void);
extern void ST_AGPS_TST_sagpsmgr_suspend( const ST_AGPS_suspend_t);

// ephs manager related
extern gnss_error_t ST_AGPS_TST_ephmgr_dump_eph(const satid_t, ephemeris_raw_t *);
extern gnss_error_t ST_AGPS_TST_ephmgr_fill_eph(const satid_t, ephemeris_raw_t *);
extern void ST_AGPS_TST_ephmgr_set_real_ephemeris_update(const tBool);
extern void ST_AGPS_TST_ephmgr_set_predicted_ephemeris_update(const tBool);

// polys manager related
extern gnss_error_t ST_AGPS_TST_sagpsmgr_dump_poly(const tS32, const tS32, tU32 *, void *, tS32 *);
extern gnss_error_t ST_AGPS_TST_sagpsmgr_dump_seed(const tS32, tU32 *, void *, tS32 *);
extern gnss_error_t ST_AGPS_TST_sagpsmgr_fill_poly(const satid_t, const tS32, void *);
extern void ST_AGPS_TST_dump_predicted_eph(void);

// polys and seeds manager related

// predicted ephemeris related
extern void ST_AGPS_TST_keplerize(gnsslibid_t, ephemeris_raw_t*);

#endif /* _ST_AGPS_TESTING_H */
