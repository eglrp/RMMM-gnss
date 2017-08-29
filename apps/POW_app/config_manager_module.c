/*
 * config_manager_module.c
 *
 *  Created on: 24 juil. 2017
 *      Author: jsanchez
 */

#include "gnss_data.h"
#include "gnssapp.h"      /* to configure power services  */
#include "gnss_debug.h"
#include "gnss_api.h"     /* to use GNSS api              */
#include "gps_nvm.h"
#include "POW_app.h"
#include "libs/cJSON.h"
#include "clibs.h"

void apply_new_config(){

  gnss_low_power_periodic_mode_t  periodic;
  gnss_low_power_cyclic_mode_t  cyclic;
  gnss_app_lowpow_standby_type_t Standby;

  _clibs_memset(&periodic,0,sizeof(gnss_low_power_periodic_mode_t));

  /* set periodic mode at period of 10 seconds */
  periodic.periodic_mode = TRUE;                  /* < Activate periodic mode           */
  periodic.RTC_refresh = 0;                       /* < No RTC refresh                   */
  periodic.EPH_refresh = 1;                       /* < Ephemeris refresh                */
  periodic.NoFixOffTime = 10;                     /* < wait fix during 60 seconds       */
  periodic.NoFixTimeout = 10;                     /* < if no fix retry after 60 seconds */
  periodic.fix_on_time = 1;                       /* < wait 1 fix                       */
  periodic.fix_period = config.fix_period_s; /* < provide a fix every 25 seconds   */

  /* disable cyclic mode */;
  _clibs_memset(&cyclic,0,sizeof(gnss_low_power_cyclic_mode_t));

  Standby = GNSSAPP_LOW_POWER_STANDBY_ENABLE;

  gnssapp_low_power_setup_update( Standby , &cyclic, & periodic );               /* periodic setup */

  GPS_DEBUG_MSG(( "[CLOE_demo] Setting fix_period_s to %d and report_every_n_fixes to %d\r\n", config.fix_period_s, config.report_every_n_fixes));

  nvm_status_t status;
  void *test ;
  nvm_status_t status2 ;
  Config result;
  nvm_status_t status3;
  char *p ;

  GPS_DEBUG_MSG( ( "[CLOE_demo] Storing in the NVM the new config \r\n"));
  status = nvm_create(128,1,1,sizeof(Config));

  test = &config;
  status2 = nvm_write(128, 1, test);

  status3 = nvm_copy(128,1, &result);


}

void set_new_config(char * json){
  cJSON * root = cJSON_Parse(json);
  cJSON *included = cJSON_GetObjectItemCaseSensitive(root, "included");
  cJSON *data = cJSON_GetObjectItemCaseSensitive(root, "data");

  if(included) // Coming for new config
  {
    cJSON *attributes = cJSON_GetArrayItem(included, 0);
    cJSON *attributes_item = cJSON_GetObjectItemCaseSensitive(attributes, "attributes");
    cJSON *config_item = cJSON_GetObjectItemCaseSensitive(attributes_item, "config");
    cJSON *report_every_n_fixes_item = cJSON_GetObjectItemCaseSensitive(config_item, "report_every_n_fixes");
    cJSON *fix_period_s_item = cJSON_GetObjectItemCaseSensitive(config_item, "fix_period_s");
    cJSON *fleet_id_item = cJSON_GetObjectItemCaseSensitive(attributes_item, "fleet");
    cJSON *imei_item = cJSON_GetObjectItemCaseSensitive(attributes_item, "imei");

    if (cJSON_IsNumber(report_every_n_fixes_item)){
      config.report_every_n_fixes = report_every_n_fixes_item->valueint;
    }
    if (cJSON_IsNumber(fix_period_s_item))
    {
      config.fix_period_s = fix_period_s_item->valueint;
    }
    if (cJSON_IsNumber(fleet_id_item))
    {
      sprintf(config.fleet_id,"%.0f\0",fleet_id_item->valuedouble);
    }
  }
  else if(data) // Coming from tracker activation
  {
    cJSON *attributes_item = cJSON_GetObjectItemCaseSensitive(data, "attributes");
    cJSON *config_item = cJSON_GetObjectItemCaseSensitive(attributes_item, "config");
    cJSON *report_every_n_fixes_item = cJSON_GetObjectItemCaseSensitive(config_item, "report_every_n_fixes");
    cJSON *fix_period_s_item = cJSON_GetObjectItemCaseSensitive(config_item, "fix_period_s");
    cJSON *fleet_id_item = cJSON_GetObjectItemCaseSensitive(attributes_item, "fleet");
    cJSON *imei_item = cJSON_GetObjectItemCaseSensitive(attributes_item, "imei");

    if (cJSON_IsNumber(report_every_n_fixes_item))
    {
      config.report_every_n_fixes = report_every_n_fixes_item->valueint;
    }
    if (cJSON_IsNumber(fix_period_s_item))
    {
      config.fix_period_s = fix_period_s_item->valueint;
    }
    if (cJSON_IsNumber(fleet_id_item))
    {
      sprintf(config.fleet_id,"%.0f\0",fleet_id_item->valuedouble);
    }
    GPS_DEBUG_MSG(( "[CLOE_demo] Tracker activated with fleet = %s\r\n", config.fleet_id));
  }
  free(json);
  cJSON_Delete(root);
  apply_new_config();
}
