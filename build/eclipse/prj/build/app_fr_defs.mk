#
# assembler defs for gnssapp projects
#

#
# compiler defs for gnssapp projects
#

PROJ_CDEFS:=\
  __MST__                           \
  __GPS_SINGLE_CORE__               \
  __$(call MK_UPPER,${MOD_CORE})__  \
  UART_DEBUG                        \
  ${PROJ_MOD_OS}

PROJ_CINCDIRS:=\
  clibs                             \
  os_svc                            \
  os_svc/adc                        \
  os_svc/can                        \
  os_svc/fsmc                       \
  os_svc/fsw                        \
  os_svc/gpio                       \
  os_svc/i2c                        \
  os_svc/mcu                        \
  os_svc/msp                        \
  os_svc/mtu                        \
  os_svc/pwr                        \
  os_svc/sdi                        \
  os_svc/sqi                        \
  os_svc/ssp                        \
  os_svc/uart                       \
  os_svc/usb                        \
  modules/gnssapp_plugins           \
  modules/in_out                    \
  modules/nmea                      \
  modules/sdlog                     \
  modules/fat                       \
  modules/can                       \
  modules/xtal                      \
  modules/modem                     \
  ${MOD_CORE}/platforms             \
  ${MOD_CORE}/frontend              \
  ${MOD_CORE}/gpsapp

#
# linker defs for gnssapp projects
#

# Scatter file defs
PROJ_SCF_DEFS:=\
  -DLR_CODE_BASE=${LR_CODE_BASE}        \
  -DLR_CODE_SIZE=${LR_CODE_SIZE}        \
  -DDATA_TCM_START=${DATA_TCM_START}    \
  -DDATA_TCM_SIZE=${DATA_TCM_SIZE}      \
  -DOS_HEAP_AREA_START=${OS_HEAP_BASE}  \
  -DOS_HEAP_AREA_SIZE=${OS_HEAP_SIZE}   \

# Libraries to link
LIBNAMES+=debug
LIBNAMES+=${OS_LIBNAMES}
LIBNAMES+=lld
LIBNAMES+=common
