#
# Check configuration
#

ifeq (${PROJ_CFG},)
$(error PROJ_CFG '${PROJ_PACKAGE}' not allowed. Availabile values: $(foreach cfgfile,$(wildcard configs/*.mk),$(notdir $(basename ${cfgfile}))))
endif

ifeq ($(findstring ${PROJ_MOD_OS},${PROJ_OS_LIST}),)
$(error PROJ_MOD_OS '${PROJ_MOD_OS}' not allowed. Availabile values: ${PROJ_OS_LIST})
endif

ifeq ($(findstring ${PROJ_PACKAGE},${PROJ_PACKAGE_LIST}),)
$(error PROJ_PACKAGE '${PROJ_PACKAGE}' not allowed. Availabile values: ${PROJ_PACKAGE_LIST})
endif

ifeq ($(findstring ${PROJ_LOAD_REGION},${PROJ_LOAD_REGION_LIST}),)
$(error PROJ_LOAD_REGION '${PROJ_LOAD_REGION}' not allowed. Availabile values: ${PROJ_LOAD_REGION_LIST})
endif

ifneq ($(findstring ${PROJ_MOD_APP},${PROJ_GNSSBASED_LIST}),)
ifeq ($(findstring ${PROJ_NVM_REGION},${PROJ_NVM_REGION_LIST}),)
$(error PROJ_NVM_REGION '${PROJ_NVM_REGION}' not allowed. Availabile values: ${PROJ_NVM_REGION_LIST})
endif
endif
