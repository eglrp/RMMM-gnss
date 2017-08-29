################################################################################
#
# Standard set of files for the project
#
################################################################################

# Files needed in all gnssapp configurations
CSOURCES+=modules/fat/ff.c
CSOURCES+=modules/fat/syscall.c
CSOURCES+=modules/gnssapp_plugins/gnssapp_plugins.c
CSOURCES+=modules/gnssapp_plugins/nmea_ext_plugin.c
CSOURCES+=modules/gnssapp_plugins/rtcm_plugin.c
CSOURCES+=modules/gnssapp_plugins/stagps_plugin.c
CSOURCES+=modules/gnssapp_plugins/waas_plugin.c
CSOURCES+=modules/in_out/in_out.c
CSOURCES+=modules/nmea/nmea.c
CSOURCES+=modules/nmea/nmea_support.c
CSOURCES+=modules/sdlog/sdlog.c
CSOURCES+=modules/shutdn/shutdn_ctrl.c
CSOURCES+=apps/POW_app/tools.c
CSOURCES+=apps/POW_app/at_module.c
CSOURCES+=apps/POW_app/connection_manager.c
CSOURCES+=apps/POW_app/http_module.c
CSOURCES+=apps/POW_app/config_manager_module.c
CSOURCES+=apps/POW_app/libs/cJSON.c
CSOURCES+=apps/POW_app/libs/picohttpparser.c
CSOURCES+=apps/POW_app/libs/mqtt/MQTTPacket.c
CSOURCES+=apps/POW_app/libs/mqtt/MQTTConnectClient.c
CSOURCES+=os_svc/adc/svc_adc.c
CSOURCES+=os_svc/can/svc_can.c
CSOURCES+=os_svc/fsmc/svc_fsmc.c
CSOURCES+=os_svc/fsw/svc_fsw.c
CSOURCES+=os_svc/gpio/svc_gpio.c
CSOURCES+=os_svc/i2c/svc_i2c.c
CSOURCES+=os_svc/mcu/svc_mcu.c
CSOURCES+=os_svc/msp/svc_msp.c
CSOURCES+=os_svc/mtu/svc_mtu.c
CSOURCES+=os_svc/pwr/svc_pwr.c
CSOURCES+=os_svc/sdi/svc_sdi.c
CSOURCES+=os_svc/sqi/svc_sqi.c
CSOURCES+=os_svc/ssp/svc_ssp.c
CSOURCES+=os_svc/uart/svc_uart.c
CSOURCES+=os_svc/uart/lld_uart.c  #To be deleted in the final release
CSOURCES+=os_svc/usb/svc_usb.c
CSOURCES+=os_svc/svc_ver.c
CSOURCES+=${MOD_CORE}/frontend/frontend.c
CSOURCES+=${MOD_CORE}/gpsapp/gnssapp.c
CSOURCES+=${MOD_CORE}/gpsapp/main_demo_fr.c
CSOURCES+=${MOD_CORE}/gpsapp/retarget.c
CSOURCES+=${MOD_CORE}/platforms/platform_common.c
CSOURCES+=${MOD_CORE}/platforms/${MOD_PCKG}/platform.c

ASMSOURCES+=${MOD_CORE}/gpsapp/gnu/crt0_FR.asm
