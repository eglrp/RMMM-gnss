/*****************************************************************************
   FILE:          <filename>
   PROJECT:       <project name>
   SW PACKAGE:    <software package name>
------------------------------------------------------------------------------
   DESCRIPTION:   <...>
------------------------------------------------------------------------------
   COPYRIGHT:     (c) <year> STMicroelectronics, (<group>) <site>
------------------------------------------------------------------------------
   Developers:
      AI:   Author Initials
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
   ----------+------+------------------------------------------------------
   yy.mm.dd  |  AI  | Original version
*****************************************************************************/

#ifndef SVC_MCU_H
#define SVC_MCU_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "defines.h"
#include "macros.h"
#include "typedefs.h"

#include "lld.h"
#include "lld_vic.h"
#include "lld_uart.h"
#include "lld_mtu.h"

#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#if (__ARMCC_VERSION >= 300586) || defined( __GNUC__)
#define FLASH_MODIFY          __attribute__((section ("FLASH_MODIFY_REGION")))
#else
#define FLASH_MODIFY
#endif

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

/**< Type for peripheral ID */
typedef enum svc_mcu_per_id_s
{
  SVC_MCU_PER_ID_MTU,
  SVC_MCU_PER_ID_UART,
  SVC_MCU_PER_ID_UART_HW_FLOW_CTRL,
  SVC_MCU_PER_ID_SSP,
  SVC_MCU_PER_ID_SDI,
  SVC_MCU_PER_ID_CAN,
  SVC_MCU_PER_ID_I2C,
  SVC_MCU_PER_ID_USB,
  SVC_MCU_PER_ID_MSP,
  SVC_MCU_PER_ID_GPIO,
  SVC_MCU_PER_ID_ADC,
  SVC_MCU_PER_ID_NUMBER
} svc_mcu_per_id_t;

typedef enum svc_mcu_cpu_speed_s
{
  SVC_MCU_CPU_SPEED_LOW,
  SVC_MCU_CPU_SPEED_HIGH
} svc_mcu_cpu_speed_t;

/**< Type for peripheral physical address */
typedef tUInt         svc_mcu_addr_t;

/**< Type for peripheral IRQ line */
typedef tU8           svc_mcu_irq_line_t;

/**< Type for physical information of each peripheral */
typedef struct
{
  const tU32                 number;
  const svc_mcu_addr_t *     addr;
  const svc_mcu_irq_line_t * irq_line;
} svc_mcu_phy_item_t;

struct svc_mcu_item_s;

/**< Type of available generic service commands */
typedef enum svc_mcu_cmd_id_s
{
  SVC_MCU_CMD_ID_CHANGE_SPEED,
  SVC_MCU_CMD_ID_SUSPEND_TRANSFER,
  SVC_MCU_CMD_ID_RESTORE_TRANSFER,
  SVC_MCU_CMD_ID_CHECK_DATA_RATE_CHANGE
} svc_mcu_cmd_id_t;

/**< Type of command callback for each peripheral service */
typedef gpOS_error_t  (*svc_mcu_cmdcallback_t)( svc_mcu_cmd_id_t cmd_id, void *param);

/**< Service item type */
typedef struct svc_mcu_item_s
{
  struct svc_mcu_item_s *     next;         /**< Next service in the list */

  svc_mcu_per_id_t            srv_id;       /**< Service ID of this item */

  tUInt                       bus_id;       /**< Bus clock id */
  gpOS_partition_t *          part;         /**< Memory partition used for dynamic allocated data */
  tU32                        mem_used;     /**< Dynamic allocated memory used by this service */
  const svc_mcu_phy_item_t *  phy_item;     /**< Pointer to related physical info */

  svc_mcu_cmdcallback_t       cmdif;        /**< API to be called to issue service commands */
} svc_mcu_item_t;

typedef tU32 svc_mcu_corefreqcfg_t;

typedef tU32 svc_mcu_clksrc_t;

typedef tU32 svc_mcu_busfreq_t;

/**< MCU system timer configuration type */
typedef struct
{
  tS32                    factor;
  LLD_MTU_PrescalerTy     prescaler;
  tU32                    ext_freq;
} svc_mcu_systimercfg_t;

/**< MCU clock configuration type */
typedef struct
{
  svc_mcu_corefreqcfg_t   corefreqid;
  svc_mcu_clksrc_t        clk_source;
  tU16                    clk_divider;
  boolean_t               clk_48f0_enabled;
  tU8                     clk_arm_speed;  //Dummy parameter to keep backward compatibility
  svc_mcu_busfreq_t       bus_speed;
  svc_mcu_systimercfg_t   systimer_cfg;
} svc_mcu_clkcfg_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tUInt            svc_mcu_get_ports      ( svc_mcu_per_id_t per_id);
extern void             svc_mcu_enable         ( svc_mcu_per_id_t per_id, tUInt port_num);
extern void             svc_mcu_disable        ( svc_mcu_per_id_t per_id, tUInt port_num);
extern tUInt            svc_mcu_get_addr       ( svc_mcu_per_id_t per_id, tUInt port_num);
extern tUInt            svc_mcu_get_irq_line   ( svc_mcu_per_id_t per_id, tUInt port_num);
extern void             svc_mcu_enter_wfi      ( void);
#if defined( __STA8090__) && !defined( RCIF_OVER_UART)
extern void             svc_mcu_exec_wfi       ( const gpOS_syscall_param_t);
#endif
extern void             svc_mcu_sw_reset       ( void);
extern const tChar *    svc_mcu_getprodname    ( void);
extern tUInt            svc_mcu_getromver      ( void);
extern tUInt            svc_mcu_getcpuusage    ( void);

extern gpOS_error_t     svc_mcu_init              ( gpOS_partition_t *part, const svc_mcu_corefreqcfg_t mclkid);
extern gpOS_error_t     svc_mcu_install           ( const svc_mcu_per_id_t per_id, const tU32 busclk_id, svc_mcu_item_t *item_ptr);
extern gpOS_error_t     svc_mcu_uninstall         ( const svc_mcu_per_id_t per_id);
extern gpOS_error_t     svc_mcu_busclk_set        ( const tU32 id, tU32 freq);
extern gpOS_error_t     svc_mcu_update_data_rates ( const tU32 id);
extern gpOS_error_t     svc_mcu_suspend_transfers ( const tU32 id);
extern gpOS_error_t     svc_mcu_restore_transfers ( const tU32 id);
extern gpOS_error_t     svc_mcu_busclk_get        ( const tU32 id, tU32 *freq_ptr);

extern gpOS_error_t     svc_mcu_check_data_rates_feasibility  ( const tU32 id, const tU32 freq);

extern void             svc_mcu_changeclockmode   ( tUInt mode);
extern void             svc_mcu_RFenable          ( tBool enable);
extern tUInt            svc_mcu_setRIOSCconfig    ( tUInt choice, tUInt param1, tUInt param2);
extern tUInt            svc_mcu_getRIOSCfrequency ( tU32* trim_out);
extern tVoid            svc_mcu_setPLL            ( tBool enable);
#endif /* SVC_MCU_H */
// End of file
