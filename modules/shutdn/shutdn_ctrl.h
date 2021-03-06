//!
//!   \file     lvd_mgmt.h
//!   \brief    <i><b> Low Voltage Detector Management header file</b></i>
//!   \author   Aldo Occhipinti
//!   \version  1.0
//!   \date     2014.03.14
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef _SHUTDN_MGMT_H_
#define _SHUTDN_MGMT_H_

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"

#ifdef __cplusplus
extern "C" {
#endif

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

extern gpOS_error_t   shutdn_ctrl_start          ( void);

#endif  // _LVD_MGMT_H_

// End of file
