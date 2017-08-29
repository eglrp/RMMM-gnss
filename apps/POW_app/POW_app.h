/*********************************************************************************************
   FILE:          POW_app.h
----------------------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2016 STMicroelectronics
 **********************************************************************************************/

#include "gnss_data.h"

#define POSITIONS_ARRAY_SIZE 15
#define PDN 2
#define SOCKET_ID 1
#define _AT(x) x"\r\n"

typedef struct pow_manager_t
{
    gpOS_wakelockid_t          wakelock_id;
    boolean_t             standby_wakeup;

}pow_manager_t;

typedef struct Config
{
    tUInt fix_period_s;
    tUInt report_every_n_fixes;
    char imei[16];
    char fleet_id[17];

}Config;

extern void pow_app_init(gpOS_partition_t *part);
extern Config config;
extern tUInt COM_PORT;
extern tUInt MAX_READ_BUFFER_SIZE;
extern Gnss_data    positions[POSITIONS_ARRAY_SIZE];
extern tUInt        n_positions;
extern tUInt        testing;
extern Gnss_nvm gnss_data;
