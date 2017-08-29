#ifndef RTC_DRV_H
#define RTC_DRV_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"
#include "gnss_defs.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*{{{  rtc trim defines*/
#define RTC_TRIM_MODE_MEAS_FULL  (0U)
#define RTC_TRIM_MODE_RESET      (1U)
#define RTC_TRIM_MODE_MEAS_PART1 (2U)
#define RTC_TRIM_MODE_MEAS_PART2 (3U)
/*}}}  */

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*{{{  rtc_validity_t*/
typedef enum
{
  NOT_VALID,
  VALID
} rtc_drv_validity_t;
/*}}}  */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern void           rtc_drv_init                ( void);

extern gnss_error_t   rtc_drv_rtt_init            ( void);
extern gnss_error_t   rtc_drv_rtt_read            ( tUInt *, gpOS_clock_t *);
extern gnss_error_t   rtc_drv_rtt_write           ( gpOS_clock_t *);
extern boolean_t      rtc_drv_rtt_is_enable       ( void );
extern boolean_t      rtc_drv_rtt_is_initialize   ( void );

extern gnss_error_t   rtc_drv_rtc_init            ( void);
extern gnss_error_t   rtc_drv_rtc_read            ( tUInt *);
extern gnss_error_t   rtc_drv_rtc_write           ( tUInt );
extern gnss_error_t   rtc_drv_get_utc_time        ( tUInt *,rtc_drv_validity_t *);
extern gnss_error_t   rtc_drv_rtc_accurate_write  ( tUInt *rtc_int_val, tUInt *rtc_fra_val, gpOS_clock_t *cpu_time, tDouble accurate_tps);
extern gnss_error_t   rtc_drv_rtc_accurate_read   ( tUInt *rtc_int_val, tUInt *rtc_fra_val, gpOS_clock_t *cpu_time, tDouble accurate_tps);
extern gnss_error_t   rtc_drv_rtc_test            ( tUInt, tUInt *, tUInt *, tUInt *, tUInt *);

extern gnss_error_t   rtc_drv_rtc_trim            ( tUInt, tUInt, tDouble *);
#endif /* _RTC_DRV_H_ */
