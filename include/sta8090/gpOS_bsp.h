/**
 * @file    gpOS_bsp.h
 * @brief   API of a standard BSP for OS20.
 *
 * @addtogroup OS20
 */

#ifndef gpOS_BSP_H
#define gpOS_BSP_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

extern tU32           gpOS_bsp_heap_area_base;
extern tU32           gpOS_bsp_stack_area_base;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern void           gpOS_bsp_reset_handler          ( void);

extern void           gpOS_bsp_mcu_init               ( tVoid);
extern gpOS_error_t   gpOS_bsp_mcu_setclkcfg          ( tU32 config, boolean_t forced_mode);
extern tU32           gpOS_bsp_mcu_getclkcfg          ( tVoid);
extern tU32           gpOS_bsp_mcu_getpwrlowfreqid    ( tVoid);
extern tU32           gpOS_bsp_mcu_getresetclkcfg     ( tVoid);

extern void           gpOS_bsp_timer_init             ( void *bsp_timer_cfg_ptr);
extern void           gpOS_bsp_timer_set_clock        ( void *cfg_ptr, boolean_t update_value);
extern void           gpOS_bsp_timer_reset_timeout    ( const gpOS_clock_t timeout, boolean_t timer_switch_off);
extern void           gpOS_bsp_timer_update_timeout   ( const tS32 frequency_ratio, const tU32 ratio_scale);
extern gpOS_clock_t   gpOS_bsp_timer_time_now         ( void);
extern void           gpOS_bsp_timer_set_timeslice    ( gpOS_bool_t new_status);
extern tUInt          gpOS_bsp_timer_get_clock_speed  ( void);
extern void           gpOS_bsp_timer_start            ( void);
extern void           gpOS_bsp_timer_stop             ( void);
extern void           gpOS_bsp_timer_get_rtc_time     ( tU32 *Int, tU32 *Frac);

extern void           gpOS_bsp_interrupt_handler_init ( void);
extern void           gpOS_bsp_interrupt_dispatch_irq ( gpOS_interrupt_line_t *line, gpOS_interrupt_callback_t *callback_func_ptr, void **callback_param_ptr);
extern void           gpOS_bsp_interrupt_clear_irq    ( gpOS_interrupt_line_t line);
extern void           gpOS_bsp_interrupt_install      ( gpOS_interrupt_line_t line, gpOS_interrupt_priority_t priority, gpOS_interrupt_callback_t callback_func, void *callback_param);
extern void           gpOS_bsp_interrupt_uninstall    ( gpOS_interrupt_line_t line);
extern void           gpOS_bsp_interrupt_enable       ( gpOS_interrupt_line_t line);
extern void           gpOS_bsp_interrupt_disable      ( gpOS_interrupt_line_t line);

#if 0
extern void           gpOS_bsp_interrupt_lock         ( void);
extern void           gpOS_bsp_interrupt_unlock       ( void);
#endif

#if defined( VFP_SUPPORT)
extern void           gpOS_bsp_fp_init                ( void);
#endif

#endif /* gpOS_BSP_H */
