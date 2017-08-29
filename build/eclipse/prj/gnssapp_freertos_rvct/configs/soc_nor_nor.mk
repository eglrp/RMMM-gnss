################################################################################
#
# GNSS application configuration file
#
################################################################################

#
# Type of project
#
PROJ_MOD_APP      :=  gnssapp

#
# Suffix to be added to this configuration
#
PROJ_MOD_EXTRA    :=

#
# Package can be:
# - SOC for STA80xxEX and its derivatives
# - SAL for STA80xxF and STA80xxG and their derivatives
PROJ_PACKAGE      :=  SOC

#
# Load region is the region of memory where the binary produced will be loaded
# It can be:
# - SQI
# - NOR
# - RAM
PROJ_LOAD_REGION  :=  NOR

#
# NVM region is the region of memory where the GNSS NVM data  will be saved
# It can be:
# - SQI
# - NOR
# - RAM
PROJ_NVM_REGION   :=  NOR

#
# Optional settings.
# Some settings are defined by default in specific makefiles. If you want to
# apply specific settings, use this section to uncomment and define them.
# Check documentation for explanation of each setting and allowed values.
#
PROJ_OPT_FEATURES     := SBAS
# PROJ_FAST_ENABLE      :=
# PROJ_ENTRY_POINT      :=
# DATA_TCM_START        :=
# DATA_TCM_SIZE         :=
# OS_HEAP_BASE          :=
# OS_HEAP_SIZE          :=
# PROJ_SCFFILE_BASENAME :=
#

################################################################################
#
# This part should not be modified
#
################################################################################

#
# Operating system used in this project
#
PROJ_MOD_OS       :=  FREERTOS

#
# Toolchain used in this project
#
PROJ_TC           :=  rvct

#
# Toolchain variant used for this project
# It can be:
# - DS5     for ARM DS 5.xx and RVCT 5.xx. Check installation path variable,
#           DS5_PATH_INST.
# - RVDS41  for ARM RVDS 4.1. Check installation path variable, RVDS_PATH_INST.
# - RVDS40  for ARM RVDS 4.0. Check installation path variable, RVDS_PATH_INST.
# - RVDS31  for ARM RVDS 3.1. Check installation path variable, RVDS_PATH_INST.
#
PROJ_COMPILER     :=  DS5
