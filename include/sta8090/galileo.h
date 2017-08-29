/*{{{  COMMENT Standard Header*/

/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995-2012
                   All Rights Reserved

Source file name : galileo.h
Description:       Galileo Library

Date              Modification
23  Sep 12        Created

************************************************************************/
/*}}}  */

#ifndef GALILEO_H
#define GALILEO_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "gnss_defs.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define GALILEO_SEC_CODE_LENGTH                     25

#define GALILEO_SYMS_PER_FRAME                      250  // 4ms symbols, before Viterbi
#define GALILEO_RANGE_MODULUS                       200  // ms
#define GALILEO_PACKER_BITS_PER_WORD                30
#define GALILEO_PACKER_BITS_PER_SUBFRAME            240  // raw, before Viterbi, excluding 10-bit sync word
#define GALILEO_PACKER_WORDS_PER_SUBFRAME           8    // raw, before Viterbi
#define GALILEO_PACKER_MASK_WORD                    0x3FFFFFFF  // 30-bit
#define GALILEO_PACKER_BITS_PER_MESSAGE             240  // payload even+odd after Viterbi
#define GALILEO_PACKER_WORDS_PER_MESSAGE            8    // payload even+odd after Viterbi
#define GALILEO_PACKER_RESET_IND_MASK               0x80000000U  // packer reset indication bit

#define GALILEO_INIT_WEEK                           1024  // GPS Week at Sunday 1999 Aug 22nd UTC 00:00
#define GALILEO_WEEK_VALID(week)                    ((((week) >= MIN_WEEK_N) && ((week) < MAX_WEEK_N))     ? TRUE:FALSE)
#define GALILEO_TOW_VALID(tow)                      ((((tow) >= 0.0) && ((tow) <= (tDouble)SECS_PER_WEEK)) ? TRUE:FALSE)
#define GALILEO_TIME_VALID(time)                    (((GALILEO_WEEK_VALID((time).week_n) == TRUE) && (GALILEO_TOW_VALID((time).tow) ==TRUE)) ? TRUE:FALSE)

#define GALILEO_EXTRACT_WORD_TYPE(word)             (((word)[0] >> 26) & 0x3F)
#define GALILEO_EXTRACT_ALMANAC_IODA(word)          (((word)[0] >> 22) & 0xF)
#define GALILEO_EXTRACT_ALMANAC_SVID1(word)         (((word)[0] >>  4) & 0x3F)
#define GALILEO_EXTRACT_ALMANAC_SVID2(word)         (((word)[1] >> 15) & 0x3F)
#define GALILEO_EXTRACT_ALMANAC_SVID3(word)         (((word)[2] >> 19) & 0x3F)

#define GALILEO_IS_PACKER_RESET(word)               ((((word)[0]) & (GALILEO_PACKER_RESET_IND_MASK)) ? TRUE : FALSE)

#define GALILEO_SUBFRAME_HAS_TIME(word_type)        ((((word_type)==0) || ((word_type)==5) || ((word_type)==6)) ? TRUE:FALSE)
#define GALILEO_SUBFRAME_HAS_EPHEMERIS(word_type)   ((((word_type)>=1) && ((word_type)<=5))  ? TRUE:FALSE)
#define GALILEO_SUBFRAME_HAS_ALMANAC(word_type)     ((((word_type)>=7) && ((word_type)<=10)) ? TRUE:FALSE)
#define GALILEO_SUBFRAME_HAS_IONO(word_type)        (((word_type)==5)  ? TRUE:FALSE)
#define GALILEO_SUBFRAME_HAS_UTC(word_type)         (((word_type)==6)  ? TRUE:FALSE)
#define GALILEO_SUBFRAME_HAS_GGTO(word_type)        (((word_type)==10) ? TRUE:FALSE)

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef struct {
  tUInt   word[GALILEO_PACKER_WORDS_PER_SUBFRAME];
} galileo_subframe_data_t;

/*{{{  subframe_payload_data_t*/
typedef struct galileo_subframe_payload_data_tag
{
  gpOS_clock_t        cpu_time;

  tUInt               avail     : 2;   // 0 = none   1 = odd    2 = even   3 = odd+even
  tUInt               page_type : 1;   // 0 = nominal  1=alert
  tUInt               crc_ok    : 1;   // 0 = CRC ERROR  1=OK
  tUInt               spare0    : 28;

  master_timebase_t   mtb_time;
  tUInt               word[GALILEO_PACKER_WORDS_PER_MESSAGE];

} galileo_subframe_payload_data_t;
/*}}}  */

/*{{{  galileo_ephemeris_raw_t*/
typedef struct galileo_ephemeris_raw_tag
{
  tUInt week                : 16;
  tUInt toe                 : 14;
  tUInt ephems_n            : 2;

  tUInt toc                 : 14;
  tUInt iod_nav             : 10;
  tUInt SISA                : 8;

  tUInt age_h               : 10;
  tUInt BGD_E1_E5a          : 10;
  tUInt BGD_E1_E5b          : 10;
  tUInt E1BHS               : 2;

  tUInt inclination         : 32;
  tUInt eccentricity        : 32;
  tUInt root_a              : 32;
  tUInt mean_anomaly        : 32;
  tUInt omega_zero          : 32;
  tUInt perigee             : 32;

  tUInt i_dot               : 14;
  tUInt available           : 1;
  tUInt health              : 1;
  tUInt motion_difference   : 16;

  tUInt crs                 : 16;
  tUInt crc                 : 16;
  tUInt cus                 : 16;
  tUInt cuc                 : 16;
  tUInt cis                 : 16;
  tUInt cic                 : 16;

  tUInt omega_dot           : 24;
  tUInt SVID                : 6;
  tUInt E1BDVS              : 1;
  tUInt predicted           : 1;

  tUInt af2                 : 6;
  tUInt af1                 : 21;
  tUInt word_available      : 5;

  tUInt af0                 : 31;
  tUInt spare0              : 1;

  tUInt time_distance_h     : 6;
  tUInt spare1              : 26;

} galileo_ephemeris_raw_t;
/*}}}  */

/*{{{  galileo_almanac_raw_t*/
typedef struct galileo_almanac_raw_tag
{
  tUInt satid          : 16;
  tUInt svid           : 6;
  tUInt spare0         : 10;

  tUInt week           : 16;
  tUInt spare1         : 16;

  tUInt toa            : 20;
  tUInt spare2         : 12;

  tUInt delta_a        : 13;
  tUInt eccentricity   : 11;
  tUInt spare3         : 8;

  tUInt perigee        : 16;
  tUInt delta_i        : 11;
  tUInt spare4         : 5;

  tUInt omega_zero     : 16;
  tUInt omega_dot      : 11;
  tUInt spare5         : 5;

  tUInt mean_anomaly   : 16;
  tUInt af0            : 16;

  tUInt af1            : 13;
  tUInt E5b_HS         : 2;
  tUInt E1B_HS         : 2;
  tUInt ioda_1         : 4;
  tUInt ioda_2         : 4;
  tUInt word_available : 2;
  tUInt health         : 1;
  tUInt available      : 1;
  tUInt spare6         : 3;

  tUInt spare7         : 32;
  tUInt spare8         : 32;

} galileo_almanac_raw_t;
/*}}}  */

/*{{{  galileo_iono_raw_t*/
typedef struct galileo_iono_raw_tag
{
  tUInt ai0             : 11;
  tUInt ai1             : 11;
  tUInt spare0          : 10;
  tUInt ai2             : 14;
  tUInt Region1         : 1;
  tUInt Region2         : 1;
  tUInt Region3         : 1;
  tUInt Region4         : 1;
  tUInt Region5         : 1;
  tUInt available       : 1;
  tUInt spare1          : 12;

} galileo_iono_raw_t;
/*}}}  */

/*{{{  typedef*/
typedef struct galileo_iono_tag
{
  tDouble   Ai0;		    /* (sfu)						I/NAV pag5 */
  tDouble   Ai1;		    /* (sfu/degree)			I/NAV pag5 */
  tDouble   Ai2;		    /* (sfu/degree^2)   I/NAV pag5 */
  tUInt     Region1;    /* Ionospheric Disturbance Flag for region 1 */
  tUInt     Region2;    /* Ionospheric Disturbance Flag for region 2 */
  tUInt     Region3;    /* Ionospheric Disturbance Flag for region 3 */
  tUInt     Region4;    /* Ionospheric Disturbance Flag for region 4 */
  tUInt     Region5;    /* Ionospheric Disturbance Flag for region 5 */

} galileo_iono_t;
/*}}}  */

typedef struct
{
  tUInt ioda          : 4;
  tUInt svid1         : 6;
  tUInt svid2         : 6;
  tUInt svid3         : 6;
  tUInt ioda_valid    : 1;
  tUInt svid_valid    : 3;
  tUInt spare0        : 6;

  tUInt WN            : 16;  // extended from 12 --> 16 (include 1024)
  tUInt WN_valid      : 1;
  tUInt E1BHS         : 2;
  tUInt E1BDVS        : 1;
  tUInt health        : 1;
  tUInt health_valid  : 1;
  tUInt spare1        : 10;

} galileo_sync_data_t;

/*{{{  galileo_ggto_raw_t*/
typedef struct galileo_ggto_raw_tag
{
  tUInt WN0G          : 16;
  tUInt A0G           : 16;
  tUInt t0G           : 8;
  tUInt A1G           : 12;
  tUInt spare0        : 11;
  tUInt valid         : 1;

} galileo_ggto_raw_t;
/*}}}  */

/*{{{  galileo_utc_raw_t*/
typedef struct galileo_utc_raw_tag
{
  tUInt A0            : 32;
  tUInt A1            : 24;
  tUInt delta_tls     : 8;
  tUInt delta_tlsf    : 8;
  tUInt DN            : 3;
  tUInt spare0        : 5;
  tUInt tot           : 8;
  tUInt WNt           : 8;
  tUInt WNlsf         : 8;
  tUInt spare1        : 23;
  tUInt valid         : 1;

} galileo_utc_raw_t;
/*}}}  */

/*{{{  galileo_ggto_t*/
typedef struct galileo_ggto_tag
{
  tInt        WN0G;
  tDouble     t0G;
  tDouble     A0G;
  tDouble     A1G;
  boolean_t   valid;

} galileo_ggto_t;
/*}}}  */

/*{{{  galileo_ggto_mode_t*/
typedef enum {
  GALILEO_GGTO_MODE_DEFAULT = -1,
  GALILEO_GGTO_MODE_OFF     = 0,
  GALILEO_GGTO_MODE_BRDC    = 1,
  GALILEO_GGTO_MODE_EST     = 2,
  GALILEO_GGTO_MODE_AUTO    = 3

} galileo_ggto_mode_t;
/*}}}  */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif
