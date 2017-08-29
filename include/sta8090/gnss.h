/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995,1996,1997,1998,1999
                   All Rights Reserved

Source file name : gnss.h
Author :           F Pisoni

Basic GNSS definitions and functions.

Date        Modification                                    Initials
----        ------------                                    --------
26 Nov 10   Created                                         FP

************************************************************************/

#ifndef GNSS_H
#define GNSS_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "macros.h"
#include "gnss_const.h"
#include "gnss_defs.h"

#include "compass_sat_id.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*{{{  defines*/

#define GLONASS_CHANNEL_FREQ(K)       (GLONASS_TRANSMITTED_FREQ + ((K)*GLONASS_CHANNEL_SPACING))
#define GLO_L2_CHANNEL_FREQ(K)        (GLO_L2_TRANSMITTED_FREQ  + ((K)*GLO_L2_CHANNEL_SPACING))

#define TRANSMITTED_PERIOD_FP         FP_DTOFP32_DS(1e9 / GPS_TRANSMITTED_FREQ, 1)
#define WAVELENGTH_FP                 FP_DTOFP32_DS(GPS_WAVELENGTH, 1)
#define MOD_ONE_MS(val)               (rem(val,C_MS))

// SAT_ID (PRN) Assignments to GNSS Satellites
#define SAT_ID_NOT_VALID                0
#define MIN_GPS_SAT_ID                  1
#define MAX_GPS_SAT_ID                 32
#define MIN_GLONASS_SAT_ID             65
#define MAX_GLONASS_SAT_ID             92
#define MIN_GALILEO_SAT_ID            301
#define MAX_GALILEO_SAT_ID            330
#define MIN_SBAS_SAT_ID               120
#define MAX_SBAS_SAT_ID               138
#define MIN_QZSS_L1_SAIF_SAT_ID       183
#define MAX_QZSS_L1_SAIF_SAT_ID       192
#define MIN_QZSS_L1_CA_SAT_ID         193
#define MAX_QZSS_L1_CA_SAT_ID         197
#define MIN_QZSS_L1C_SAT_ID           293
#define MAX_QZSS_L1C_SAT_ID           297
#define MIN_PSEUDOLITE_SAT_ID         110
#define MAX_PSEUDOLITE_SAT_ID         119
#define MIN_L2C_SAT_ID                401
#define MAX_L2C_SAT_ID                432

#define VIRTUAL_SAT_ID                99U


// GALILEO sat_id to PRN+Pilot conversion
#define GALILEO_PRN_PILOT_OFFSET              50  // PRN convention:  1..50 == Data   51..100 == Pilot
#define GALILEO_SAT_ID_TO_PRN(sat_id,pilot)   (((pilot)==FALSE)?((sat_id)-MIN_GALILEO_SAT_ID)+1:((sat_id)-MIN_GALILEO_SAT_ID)+1+GALILEO_PRN_PILOT_OFFSET)
#define GALILEO_PRN_GET_PILOT(prn)            ((boolean_t) ((prn)>GALILEO_PRN_PILOT_OFFSET))
#define GALILEO_PRN_GET_PRN_BASE(prn)         (((prn)>GALILEO_PRN_PILOT_OFFSET)?(prn)-GALILEO_PRN_PILOT_OFFSET:(prn))

// GALILEO SV to PRN mapping
#define GALILEO_SV_TO_PRN(sv)  (sv)
#define GALILEO_PRN_TO_SV(prn) (prn)

// GLONASS PRN to channel mapping
#define MIN_GLONASS_FREQ_ID           -7
#define MAX_GLONASS_FREQ_ID           6
#define GLONASS_FREQ_ID_INVALID       0x33
#define GLONASS_CH0_SAT_ID_LOW        (MIN_GLONASS_SAT_ID - MIN_GLONASS_FREQ_ID)
#define GLONASS_CH0_SAT_ID_HIGH       ((GLONASS_CH0_SAT_ID_LOW + MAX_GLONASS_FREQ_ID + 1) - MIN_GLONASS_FREQ_ID)
#define GLONASS_NUM_CHANNELS          (MAX_GLONASS_FREQ_ID - MIN_GLONASS_FREQ_ID + 1)

#define GALILEO_CODE_LENGTH           4
#define L2C_CODE_LENGTH_MS            20

#define SAT_ID_TO_PRN(sat_id)         ((sat_id) & SIG_ID_MASK)

#define GLONASS_SAT_ID_HALF(sat_id)   ((((tInt)(sat_id)-GLONASS_CH0_SAT_ID_LOW) > MAX_GLONASS_FREQ_ID) ? 1 : 0)

#define SAT_ID_TO_K(sat_id)           ((GLONASS_SAT_ID_HALF(sat_id)==1) ? ((tInt)(sat_id)-GLONASS_CH0_SAT_ID_HIGH) : ((tInt)(sat_id)-GLONASS_CH0_SAT_ID_LOW))

#define GLONASS_K_TO_SAT_ID(K,HALF)   (((HALF)==0) ? ((K)+GLONASS_CH0_SAT_ID_LOW) : ((K)+GLONASS_CH0_SAT_ID_HIGH))

// Default sat_id allocation map
#define NUM_GPS_SAT_IDS           ((MAX_GPS_SAT_ID-MIN_GPS_SAT_ID)+1)  // GPS always linked by default
#define NUM_GALILEO_SAT_IDS       0
#define NUM_GLONASS_SAT_IDS       0
#define NUM_SBAS_SAT_IDS          ((MAX_SBAS_SAT_ID-MIN_SBAS_SAT_ID)+1)
#define NUM_QZSS_L1_SAIF_SAT_IDS  0
#define NUM_QZSS_L1_CA_SAT_IDS    0
#define NUM_QZSS_L1C_SAT_IDS      0
#define NUM_COMPASS_SAT_IDS       0
#define NUM_PSEUDOLITE_SAT_IDS    0
#define NUM_L2C_SAT_IDS           0

// Override the sat_id allocation map based on compile options
#undef  NUM_GLONASS_SAT_IDS
#define NUM_GLONASS_SAT_IDS       ((MAX_GLONASS_SAT_ID-MIN_GLONASS_SAT_ID)+1)

#undef  NUM_QZSS_L1_SAIF_SAT_IDS
#define NUM_QZSS_L1_SAIF_SAT_IDS  ((MAX_QZSS_L1_SAIF_SAT_ID-MIN_QZSS_L1_SAIF_SAT_ID)+1)

#undef  NUM_QZSS_L1_CA_SAT_IDS
#define NUM_QZSS_L1_CA_SAT_IDS    ((MAX_QZSS_L1_CA_SAT_ID-MIN_QZSS_L1_CA_SAT_ID)+1)

#undef  NUM_GALILEO_SAT_IDS
#define NUM_GALILEO_SAT_IDS       ((MAX_GALILEO_SAT_ID-MIN_GALILEO_SAT_ID)+1)

#undef  NUM_COMPASS_SAT_IDS
#define NUM_COMPASS_SAT_IDS       ((MAX_COMPASS_SAT_ID-MIN_COMPASS_SAT_ID)+1)

#define LINKED_SAT_IDS    (\
                            NUM_GPS_SAT_IDS+             \
                            NUM_GALILEO_SAT_IDS+         \
                            NUM_GLONASS_SAT_IDS+         \
                            NUM_SBAS_SAT_IDS+            \
                            NUM_QZSS_L1_SAIF_SAT_IDS+    \
                            NUM_QZSS_L1_CA_SAT_IDS+      \
                            NUM_QZSS_L1C_SAT_IDS+        \
                            NUM_COMPASS_SAT_IDS+         \
                            NUM_PSEUDOLITE_SAT_IDS+      \
                            NUM_L2C_SAT_IDS              \
                          )

#define LINKED_SAT_IDS_MASK_SIZE          (((tU32)LINKED_SAT_IDS >> 5U) + ((((tU32)LINKED_SAT_IDS & 0x1FU) != 0U) ? 1U : 0U))  // bit mask vector size for 32-bit elements

#define MIN_SAT_ID                        1
#define MAX_SAT_ID                        255

#define SAT_ID_VALID(sat_id)              (gnss_sat_id_valid(sat_id))
#define GPS_SAT_ID_VALID(sat_id)          ((((sat_id) >= (satid_t)MIN_GPS_SAT_ID)        && ((sat_id) <= (satid_t)MAX_GPS_SAT_ID)) ? TRUE:FALSE )
#define GLONASS_SAT_ID_VALID(sat_id)      ((((sat_id) >= (satid_t)MIN_GLONASS_SAT_ID)    && ((sat_id) <= (satid_t)MAX_GLONASS_SAT_ID)) ? TRUE:FALSE)
#define WAAS_SAT_ID_VALID(sat_id)         ((((sat_id) >= (satid_t)MIN_SBAS_SAT_ID)       && ((sat_id) <= (satid_t)MAX_SBAS_SAT_ID)) ? TRUE:FALSE)
#define QZSS_L1_CA_SAT_ID_VALID(sat_id)   ((((sat_id) >= (satid_t)MIN_QZSS_L1_CA_SAT_ID) && ((sat_id) <= (satid_t)MAX_QZSS_L1_CA_SAT_ID)) ? TRUE:FALSE)
#define GALILEO_SAT_ID_VALID(sat_id)      ((((sat_id) >= (satid_t)MIN_GALILEO_SAT_ID)    && ((sat_id) <= (satid_t)MAX_GALILEO_SAT_ID)) ? TRUE:FALSE)
#define QZSS_L1C_SAT_ID_VALID(sat_id)     ((((sat_id) >= (satid_t)MIN_QZSS_L1C_SAT_ID)   && ((sat_id) <= (satid_t)MAX_QZSS_L1C_SAT_ID)) ? TRUE:FALSE)
#define PSEUDOLITE_SAT_ID_VALID(sat_id)   ((((sat_id) >= (satid_t)MIN_PSEUDOLITE_SAT_ID) && ((sat_id) <= (satid_t)MAX_PSEUDOLITE_SAT_ID)) ? TRUE:FALSE)
#define L2C_SAT_ID_VALID(sat_id)          ((((sat_id) >= (satid_t)MIN_L2C_SAT_ID)        && ((sat_id) <= (satid_t)MAX_L2C_SAT_ID)) ? TRUE:FALSE)
#define SBAS_SAT_ID_VALID(sat_id)         ((((sat_id) >= (satid_t)MIN_SBAS_SAT_ID)       && ((sat_id) <= (satid_t)MAX_SBAS_SAT_ID)) ? TRUE:FALSE )

#define GLOCOM_SAT_ID_VALID(sat_id)       (((GLONASS_SAT_ID_VALID(sat_id) == TRUE) || (COMPASS_SAT_ID_VALID(sat_id) == TRUE)) ? TRUE:FALSE)

#define GALGPS_SAT_ID_VALID(sat_id)       (((GPS_SAT_ID_VALID(sat_id) == TRUE) || (GALILEO_SAT_ID_VALID(sat_id) == TRUE)) ? TRUE:FALSE)


#define WAAS_SATELLITE                    120

#define MAX_VISIBLE_LIST_LENGTH           (24)   // which value here ????


#define GNSS_COORD_FRAME_WGS84            0
#define GNSS_COORD_FRAME_IGEX98           1
#define GNSS_COORD_FRAME_MISRA96          2
#define GNSS_COORD_FRAME_PZ90_02          3
#define GNSS_COORD_FRAME_BOUCHER          4
#define GNSS_COORD_FRAME_PZ90_11          5

#define GPS_COORD_FRAME                   GNSS_COORD_FRAME_WGS84
#define GLONASS_COORD_FRAME               GNSS_COORD_FRAME_PZ90_11

#define STAGPS_MIN_SAT_ID                 ((tS32)(ST_AGPS_get_min_sat_ids()))
#define STAGPS_MAX_SAT_ID                 ((tS32)(ST_AGPS_get_max_sat_ids()))
#define STAGPS_LINKED_SAT_IDS             62 //(gnss_get_predicted_linked_sat_ids())
#define STAGPS_SAT_ID_VALID(sat_id)       (gnss_stagps_sat_id_valid(sat_id))
#define STAGPS_IDX_VALID(idx)             ((tBool)((idx) < gnss_get_predicted_linked_sat_ids()))

#define GNSSLIB_ID_NOT_VALID              255

#define CHAN_ID_NOT_VALID                 127

#define SAT_TYPE_VALID(sat_type)          ((((sat_type)>=0) && ((sat_type)<GNSS_SAT_TYPE_END)) ? TRUE:FALSE)

#define ELEVATION_SCALE                   100.0

#define STAGNSS_GPS_CONSTELLATION (0x01U)
#define STAGNSS_GLO_CONSTELLATION (0x02U)
#define STAGNSS_GAL_CONSTELLATION (0x04U)
#define STAGNSS_COM_CONSTELLATION (0x08U)

//#define STAGNSS_GNSS_SAT_ID_TO_GNSSLIB_ID(idx, sat_id)  ((boolean_t)(stagps_plugin_handler->satid_array[idx]==sat_id) ? TRUE:FALSE)
#define STAGNSS_GNSS_GNSSLIB_ID_TO_SAT_ID(idx)          (satid_t)stagps_plugin_handler->satid_array[idx]
#define STAGPS_ID_NOT_VALID 65535
/*}}}  */

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/
extern gnss_sat_type_t  gnss_sat_id_to_sat_type                           ( const satid_t sat_id);
extern satid_t          gnss_gnsslib_id_to_sat_id                         ( const gnsslibid_t idx);
extern gnsslibid_t      gnss_sat_id_to_gnsslib_id                         ( const satid_t sat_id);
extern tDouble          gnss_get_wavelength                               ( const satid_t sat_id);
extern boolean_t        gnss_sat_id_valid                                 ( const satid_t sat_id);
extern gnss_error_t     gnss_get_min_max_sat_id                           ( const gnss_sat_type_t, satid_t *, satid_t *);
extern satid_t          gnss_get_min_sat_id                               ( const gnss_sat_type_t);

extern void             gnss_init_satellite_healthy_status_forcing_mask   ( void);
extern void             gnss_set_satellite_healthy_status_forcing_mask    ( const satid_t sat_id);
extern void             gnss_clear_satellite_healthy_status_forcing_mask  ( const satid_t sat_id);
extern boolean_t        gnss_get_satellite_healthy_status_forcing_mask    ( const satid_t sat_id);
extern void             gnss_print_hex                                    ( tS8 *, const void *, const tInt);
extern tS8 *            gnss_print_hex32                                  ( tS8 *, const tUInt, const tInt);
extern tChar *          gnss_get_sat_type_name                            ( const gnss_sat_type_t);
extern tDouble          gnss_freq_to_L1                                   ( const satid_t, const tDouble);
extern tDouble          gnss_L1_to_freq                                   ( const satid_t, const tDouble);

void                    gnss_satid_mask_set_all                           ( tUInt *);
void                    gnss_satid_mask_reset                             ( tUInt *);
void                    gnss_satid_mask_setbit                            ( const satid_t, tUInt *);
void                    gnss_satid_mask_clearbit                          ( const satid_t, tUInt *);
boolean_t               gnss_satid_mask_getbit                            ( const satid_t, const tUInt *);

extern gnss_error_t     gnss_get_prn_mod_ms                               ( const gnss_prn_mod_t, tInt *);
extern tInt             gnss_sow_lminus_int                               ( const tInt, const tInt);

extern satid_t          gnss_glonass_get_alias                            ( const satid_t sat_id);
extern gnss_error_t     gnss_glonass_calc_sat_id                          ( const tUInt, const tUInt, satid_t *);
extern tDouble          gnss_glonass_freq_to_L1                           ( const satid_t, const tDouble);
extern tDouble          gnss_L1_to_glonass_freq                           ( const satid_t, const tDouble);

extern tDouble          gnss_compass_freq_to_L1                           ( const tDouble);
extern tDouble          gnss_L1_to_compass_freq                           ( const tDouble);

#endif /* GP_CONST_H */
