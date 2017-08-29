################################################################################
#
# common.mk
#
# This makefiles defines all constants used in makefiles
#
################################################################################

ifeq (${CONST_MAKE},)
#$(warning First call)
CONST_MAKE      :=  included

# Project constants
OS_FREERTOS     :=  FREERTOS
OS_GPOS         :=  OS20

# Module constants
MOD_BOOTEXT     :=  BOOT
MOD_UPGEXT      :=  UPG
MOD_BINEXT      :=  bin
MOD_LNKLSTEXT   :=  txt

#
# List of project types based on GNSS library
#
PROJ_GNSSBASED_LIST :=  binimg gnssapp

#
# List of supported packages
#
PROJ_PACKAGE_LIST     :=  SAL SOC

#
# List of supported load regions
#
PROJ_LOAD_REGION_LIST :=  SQI NOR RAM

#
# List of supported NVM regions
#
PROJ_NVM_REGION_LIST  :=  SQI NOR RAM

#
# List of supported OS
#
PROJ_OS_LIST          :=  OS20 FREERTOS

#
# Derived constants
#

# name of project
PROJ_NAME             :=  $(notdir ${CURDIR:%/=%})

# folder for project files
PROJ_PATH             :=  $(realpath ${CURDIR:%/=%})

endif
