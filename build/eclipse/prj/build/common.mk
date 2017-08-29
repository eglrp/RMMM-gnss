################################################################################
#
# common.mk
#
# This makefiles defines all variables and macros used by build makefiles
#
################################################################################

include ../build/macros.mk

################################################################################
#
# Constants section
#
################################################################################

################################################################################
#
# Variable section
#
################################################################################

#
# derived variables
#

# core recognition
MOD_ARCH:=$(strip\
  $(if $(findstring ${MOD_CORE},sta8088),arm9)     \
  $(if $(findstring ${MOD_CORE},sta8090),arm9)     \
)

# module name composition
MOD_NAME:=${MOD_CORE}_${PROJ_MOD_APP}$(if ${PROJ_MOD_EXTRA},_${PROJ_MOD_EXTRA})_${MOD_PCKG}_${MOD_LOAD_RGN}$(if ${MOD_NVM_RGN},_${MOD_NVM_RGN})

# OS related suffix
OS_SUFFIX:=$(strip\
  $(if $(findstring ${PROJ_MOD_OS},FREERTOS),fr)   \
  $(if $(findstring ${PROJ_MOD_OS},OS20),os20)     \
)

#
# folder defs
#

# folder for sdk root
PROJ_SDKROOTDIR:=$(realpath $(call MK_PATHTOUNX,${PROJ_PATH})/../../../..)

# folder for tools
PROJ_TOOLSDIR:=${PROJ_SDKROOTDIR}/build/tools

# folder for project files
PROJ_COMMONDIR:=${PROJ_SDKROOTDIR}/build/eclipse/prj

# folder for build common files
PROJ_COMMONMK:=${PROJ_COMMONDIR}/build

# folder for debugger files
PROJ_DEBUGDIR:=${PROJ_SDKROOTDIR}/build/scripts/${MOD_CORE}

# folder for configuration files
PROJ_ROOTDIR:=${PROJ_COMMONDIR}/${PROJ_NAME}

# folder for configuration files
CONFIG_ROOTDIR:=${PROJ_ROOTDIR}/configs

# folder for target files
TARGET_ROOTDIR:=${PROJ_ROOTDIR}/${PROJ_CFG}

# folder for common includes
COMMON_CINCDIR:=include

# folder for common includes
COMMON_LIBDIR:=libs

# folder for binary outputs
COMMON_BINDIR:=${PROJ_SDKROOTDIR}/bin

# objects folder
OBJS_FOLDER:=${TARGET_ROOTDIR}/objs

# Project related defines
TCDEFSINC:=${PROJ_COMMONMK}/tc_${PROJ_TC}_defs.mk

# Project related defines
PROJDEFSINC:=${PROJ_COMMONMK}/app_${PROJ_MOD_APP}_defs.mk

# Core related defines
COREDEFSINC:=${PROJ_COMMONMK}/core_${MOD_ARCH}_defs.mk

# OS related defines
OSDEFSINC:=${PROJ_COMMONMK}/os_${OS_SUFFIX}_defs.mk

# List of all incdefs files
DEFSINCLIST:=${TCDEFSINC} ${PROJDEFSINC} ${COREDEFSINC} ${OSDEFSINC}

#
# include toolchain, core and OS definitions
#
include ${TCDEFSINC}
include ${COREDEFSINC}
include ${OSDEFSINC}

# definitions for binary folder target outputs (needed in projdefsinc)
TARGET_DEST_FILENAME:=${MOD_NAME}_${OS_SUFFIX}_${PROJ_TC}
TARGET_DEST_FILENAME_EXE:=${COMMON_BINDIR}/${TARGET_DEST_FILENAME}.${TC_SUFFIX}
TARGET_DEST_FILENAME_BIN:=${COMMON_BINDIR}/${TARGET_DEST_FILENAME}.${MOD_BINEXT}
TARGET_DEST_FILENAME_LIST:=${COMMON_BINDIR}/${TARGET_DEST_FILENAME}.${MOD_LNKLSTEXT}

#
# include project type definitions
#

include ${PROJDEFSINC}

#
# define target files
#

# Configuration file
CONFIG_FILE:=${CONFIG_ROOTDIR}/${PROJ_CFG}.mk

# Assembler options file
ASMOPTS_FILE:=${TARGET_ROOTDIR}/asmopts.via

# C options file
COPTS_FILE:=${TARGET_ROOTDIR}/copts.via

# C options file
LOPTS_FILE:=${TARGET_ROOTDIR}/lopts.via

# FW configuration file
FWCFG_FILE:=${TARGET_ROOTDIR}/fwcfg.txt

# Default FW configuration file
DEF_FWCFG_FILE:=${PROJ_COMMONMK}/fwcfg_soc_default.txt

# Project makefile
PROJ_MAKEFILE:=${PROJ_ROOTDIR}/makefile

# scatter file name
SCF_TEMPLATE_FILE_NAME:=${SCF_FOLDER}/${PROJ_SCFFILE_BASENAME}_template.${TC_SCFEXT}

# scatter file name
SCF_FILE_NAME:=${TARGET_ROOTDIR}/${PROJ_SCFFILE_BASENAME}.${TC_SCFEXT}

# definitions for local folder target outputs
TARGET_SRC_FILENAME_EXE:=${TARGET_ROOTDIR}/${PROJ_NAME}.${TC_SUFFIX}
TARGET_SRC_FILENAME_LIST:=${TARGET_ROOTDIR}/${PROJ_NAME}.${MOD_LNKLSTEXT}

# definitions for binary folder target outputs
ifeq (${TARGET_POSTBUILD},)
TARGET_POSTBUILD:=${TARGET_DEST_FILENAME_BIN}
endif

# definitions for Ozone debugger
OZONE_PROJ_BASENAME:=ozn_setup
OZONE_PROJ_TEMPLATE:=${OZONE_PROJ_BASENAME}_template.jdebug
OZONE_PROJ:=${TARGET_ROOTDIR}/${OZONE_PROJ_BASENAME}.jdebug

#
# Build sources list
#
PROJ_ASMSOURCES:=${ASMSOURCES} ${EXT_ASMSOURCES}

PROJ_CSOURCES:=${CSOURCES} ${EXT_CSOURCES}

PROJ_CARMSOURCES:=${CARMSOURCES} ${EXT_CARMSOURCES}

PROJ_SOURCES:=${PROJ_ASMSOURCES} ${PROJ_CSOURCES} ${PROJ_CARMSOURCES}

#
# assembler section
#
ASMOPTS:=$(strip ${TC_ASMOPTS} ${CORE_ASMOPTS} ${PROJ_ASMOPTS} ${EXT_ASMOPTS})

ASMDEFS:=$(strip ${TC_ASMDEFS} ${PROJ_ASMDEFS} ${OS_ASMDEFS} ${EXT_ASMDEFS})

ASMINCDIRS+=$(foreach inc,$(strip ${COMMON_CINCDIR}/${MOD_CORE}/${TC_ASMTYPE} ${TC_ASMINCDIRS} ${PROJ_ASMINCDIRS} ${OS_ASMINCDIRS} ${EXT_ASMINCDIRS}),-I$(call MK_TC_PATH,${PROJ_SDKROOTDIR}/${inc}))

#
# compiler section
#

COPTS:=$(strip ${TC_COPTS} ${CORE_COPTS} ${PROJ_COPTS} ${EXT_COPTS})

CDEFS:=$(foreach def,$(strip ${TC_CDEFS} ${PROJ_CDEFS} ${OS_CDEFS} ${EXT_CDEFS}),-D${def})

CINCDIRS+=$(foreach inc,$(strip ${COMMON_CINCDIR}/${MOD_CORE} ${TC_CINCDIRS} ${PROJ_CINCDIRS} ${OS_CINCDIRS} ${EXT_CINCDIRS}),-I$(call MK_TC_PATH,${PROJ_SDKROOTDIR}/${inc}))

#
# linker section
#

# build libs list to link for RVCT
LIBOPTS:=$(strip ${TC_LIBOPTS} ${EXT_LIBOPTS})

LIBDIRS:=$(foreach inc,$(strip ${COMMON_LIBDIR} ${TC_LIBDIRS} ${PROJ_LIBDIRS} ${EXT_LIBDIRS}),$(call MK_TC_PATH,${PROJ_SDKROOTDIR}/${inc}))

LIBS:=${EXT_LIBS}
LIBS+=$(foreach lib,${LIBNAMES},${MOD_CORE}_${MOD_ARCH}_${lib}_${TC_LIBEXT})
LIBS+=${TC_LIBNAMES}

# scatter file defines
SCF_DEFS:=$(strip $(CDEFS) ${PROJ_SCF_DEFS})
