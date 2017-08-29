/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and DSP_DSP_NCO_TO_HZ_FP of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 *-----------------------------------------------------------------------------
 * Constants used in GPS library.
 *-----------------------------------------------------------------------------
 *
 ******************************************************************************/

#ifndef GP_CONST_H
#define GP_CONST_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "macros.h"
#include "mathconst.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*{{{  defines*/


#define C                          2.99792458E+8    // gps para 4.3
#define C_MS                       (C / 1000.0)     // Distance traveled by light in 1ms
#define C_INT                      ((tInt)C)         // gps para 4.3
#define C_INT_MS                   ((tInt)C_MS)      // Distance traveled by light in 1ms, calculated as integer
#define C_INT_MS_REM               (C - C_MS*1000)  // remainder of C_INT/1000
#define GPS_TRANSMITTED_FREQ       1575420000.0     // Hz
#define GLONASS_TRANSMITTED_FREQ   1602000000.0     // Hz
#define GLONASS_CHANNEL_SPACING    562500.0         // Hz
#define COMPASS_TRANSMITTED_FREQ   1561098000.0     // Hz
#define L2_TRANSMITTED_FREQ        1227600000.0     // Hz
#define GLO_L2_TRANSMITTED_FREQ    1246000000.0     // Hz
#define GLO_L2_CHANNEL_SPACING     437500.0         // Hz
#define GPS_16F0_NOMINAL_FREQ      16368000.0       // Hz
#define GNSS_F0_NOMINAL_FREQ       1023000.0        // Hz
#define EARTH_RADIUS               6378137.0        // m
#define GPS_WAVELENGTH             (C / GPS_TRANSMITTED_FREQ)
#define GLONASS_WAVELENGTH         (C / GLONASS_TRANSMITTED_FREQ)
#define COMPASS_WAVELENGTH         (C / COMPASS_TRANSMITTED_FREQ)
#define L2_WAVELENGTH              (C / L2_TRANSMITTED_FREQ)
#define STEPS_PER_CODE_CHIP        8
#define CODE_CHIPS_PER_MS          1023
#define STEPS_PER_MS               (STEPS_PER_CODE_CHIP * CODE_CHIPS_PER_MS)
#define STEPS_PER_HALF_MS          (STEPS_PER_MS / 2)
#define CARRIER_SCALE              1000
#define CARRIER_PHASE_SCALE        256

#define SECS_PER_MIN              60
#define SECS_PER_HOUR             (60 * SECS_PER_MIN)
#define SECS_PER_DAY              (24 * SECS_PER_HOUR)
#define SECS_PER_WEEK             (7 * 24 * SECS_PER_HOUR)
#define SECS_PER_HALF_WEEK        (SECS_PER_WEEK / 2)

#define WEEK_MODULO               1024
#define WEEK_HALF_MODULO          (WEEK_MODULO / 2)

#define MIN_WEEK_N_DEFAULT 1821
#define MAX_WEEK_N_DEFAULT 3300

#define U               3.986005E+14        /* earths universal gravitational param  */
#define ROOT_U          19964981.84322      /* root of earths universal gravitational param  */
#define OMEGA_ZERO_DOT  7.2921151467E-5     /* earths rotation rate radians/s           */
#define F               (-4.442807633E-10)  /* = u/c, given p A-3-16*/
#define GAMMA_12        ((77.0/60.0)*(77.0/60.0))

#define GLONASS_MU         398600.4418e9      // 398600.44e9 GM gravitational constant m^3/s^2
#define GLONASS_AE         6378136          // 6378136      // semi-major axis
#define GLONASS_J0_SQ      1082.62575e-6       // 1082.6257e-6  1082.63e-6 Second zonal harmonic of the geopotential
#define GLONASS_OMEGA      7.292115e-5   //(7.292115e-5)  // 7.292115e-5     // [rad/s] Earth rotation rate
#define GLONASS_OMEGA_SQ   (GLONASS_OMEGA*GLONASS_OMEGA)
#define GLONASS_ORBIT_C    (3.0/2.0*GLONASS_J0_SQ*GLONASS_MU*GLONASS_AE*GLONASS_AE)
#define GLONASS_C20        (-GLONASS_J0_SQ)

/* orbits constants */
#define GPS_MAX_ROOTA 5500.0  // WARNING!!!! original value: 5300.0
#define GPS_MIN_ROOTA 5000.0
#define GPS_MAX_ECCENTRICITY 0.03
#define GPS_MIN_ECCENTRICITY 0.0
#define GPS_MAX_INCLINATION  (60.0 * RADIANS)
#define GPS_MIN_INCLINATION  (50.0 * RADIANS)

// WARNING!!! The correct values shall be put here
#define GLONASS_MAX_LUNAR_SOLAR_ACC  (6.2e-6 * 2.0) // m/s^2  (2x margin respect to ICD)
#define GLONASS_MAX_ROOTA 6000.0
#define GLONASS_MIN_ROOTA 5000.0
#define GLONASS_MAX_ECCENTRICITY 0.01
#define GLONASS_MIN_ECCENTRICITY 0.0
#define GLONASS_MAX_INCLINATION  (70.0 * RADIANS)
#define GLONASS_MIN_INCLINATION  (50.0 * RADIANS)

// WARNING!!! The correct values shall be put here
#define QZSS_L1_CA_MAX_ROOTA 15300.0
#define QZSS_L1_CA_MIN_ROOTA 0.0
#define QZSS_L1_CA_MAX_ECCENTRICITY 1.03
#define QZSS_L1_CA_MIN_ECCENTRICITY 0.0
#define QZSS_L1_CA_MAX_INCLINATION  (90.0 * RADIANS)
#define QZSS_L1_CA_MIN_INCLINATION  ( 0.0 * RADIANS)


// WARNING!!! The correct values shall be put here
#define GALILEO_MAX_ROOTA 5500.0
#define GALILEO_MIN_ROOTA 5000.0
#define GALILEO_MAX_ECCENTRICITY 0.06
#define GALILEO_MIN_ECCENTRICITY 0.0
#define GALILEO_MAX_INCLINATION  (60.0 * RADIANS)
#define GALILEO_MIN_INCLINATION  (30.0 * RADIANS)


#define MAX_ALMANAC_EPHEM_TIME_DIFF   (2 * SECS_PER_WEEK)

#define MAX_ECCENTRICITY_DIFF     0.0001
#define MAX_INCLINATION_DIFF      (1.0 * RADIANS)
#define MAX_PERIGEE_DIFF          (25.0 * RADIANS)
#define MAX_OMEGA_DIFF            (25.0 * RADIANS)
#define MAX_MEAN_ANOMALY_DIFF     (25.0 * RADIANS)

#define GPS_DEFAULT_SAT_RANGE 22524252.05

#define QZSS_DEFAULT_SAT_RANGE 44262481.83

#define GLONASS_DEFAULT_SAT_RANGE 21447659.12

#define COMPASS_GEO_IGSO_DEFAULT_SAT_RANGE 37923245.47

#define COMPASS_MEO_DEFAULT_SAT_RANGE 23826842.49

#define GALILEO_DEFAULT_SAT_RANGE 25492235.88



#define CN0_52DB          (158489)
#define CN0_49DB          (79433)
#define CN0_46DB          (39811)
#define CN0_40DB          (10000)
#define CN0_37DB          (5012)
#define CN0_35DB          (3162)
#define CN0_33DB          (1995)
#define CN0_30DB          (1000)
#define CN0_29DB          (794)
#define CN0_27DB          (500)
#define CN0_25DB          (316)
#define CN0_24DB          (250)
#define CN0_23DB          (200)
#define CN0_22DB          (158)
#define CN0_21DB          (125)
#define CN0_20DB          (100)
#define CN0_18DB          (62)
#define CN0_17DB          (50)
#define CN0_16DB          (40)
#define CN0_15DB          (31)
#define CN0_14DB          (25)
#define CN0_10DB          (10)
#define CN0_5DB           (3)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* GP_CONST_H */
