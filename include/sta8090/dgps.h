/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and DSP_DSP_NCO_TO_HZ_FP of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 *-----------------------------------------------------------------------------
 * Management of bitstream to provide word based output header file.
 *-----------------------------------------------------------------------------
 *
 ******************************************************************************/

#ifndef DGPS_H
#define DGPS_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"
#include "gnss.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef tU32  (*dgps_read_t)    ( tChar *, tU32, gpOS_clock_t *);

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern const tChar *  dgps_version          ( void);
extern gnss_error_t   dgps_start            ( dgps_read_t, boolean_t);
extern gnss_error_t   dgps_add_new_data     ( const tU8 * , const tInt );
extern tInt           dgps_get_rtcm_version ( void);
extern gnss_error_t   dgps_set_rtcm_version ( const tInt);
#endif /* DGPS_H */
