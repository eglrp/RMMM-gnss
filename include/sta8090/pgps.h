/**
 * pgps.h
 * Defines the interface to the PGPS module responsible for reading the PGPS binary file
 * @author Mark Griffiths
 * @date   22/02/2009
 * Copyright (c) 2009 STMicroelectronics
 */
#ifndef PGPS_H
#define PGPS_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/* Defines and Macros */
#define PGPS_BUFFER_SIZE 2300
#define PGPS_MAX_PATH    200

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

/* Exported Function Prototypes */
extern boolean_t  pgps_load         (tU8 prn, tU8* seed, tULong seed_t0, tULong seed_tcur, tLong seed_tau_gps, tLong seed_tau_gps_dot, tU8 gps_utc_offset);
extern void       pgps_start        ( void);
extern boolean_t  pgps_generate_pwd ( tChar* vendor_id, tChar* device_id, tUInt gps_seconds, tChar* pwd);

#endif /* PGPS_H */

