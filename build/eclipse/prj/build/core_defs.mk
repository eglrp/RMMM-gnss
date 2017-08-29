################################################################################
#
# STA8090 core configuration file
#
################################################################################

ifeq (${PROJ_PACKAGE},SAL)
MOD_PCKG              :=  sal
endif
ifeq (${PROJ_PACKAGE},SOC)
MOD_PCKG              :=  soc
endif

ifeq (${PROJ_LOAD_REGION},SQI)
MOD_LOAD_RGN          :=  sqi
LR_CODE_BASE          :=  0x10000000
LR_CODE_SIZE          :=  0x00100000
LR_CODE_BASE_FWUPG    :=  0x10010010
LR_CODE_SIZE_FWUPG    :=  0x000FFFE0
endif
ifeq (${PROJ_LOAD_REGION},NOR)
MOD_LOAD_RGN          :=  nor
LR_CODE_BASE          :=  0x20000000
LR_CODE_SIZE          :=  0x00100000
LR_CODE_BASE_FWUPG    :=  0x20010010
LR_CODE_SIZE_FWUPG    :=  0x000FFFE0
endif
ifeq (${PROJ_LOAD_REGION},RAM)
MOD_LOAD_RGN          :=  ram
LR_CODE_BASE          :=  0x24000000
LR_CODE_SIZE          :=  0x00100000
endif

ifeq (${PROJ_NVM_REGION},SQI)
MOD_NVM_RGN           :=  sqi
NVM_BASE              :=  0x10100000
NVM_SIZE              :=  0x00080000
endif
ifeq (${PROJ_NVM_REGION},NOR)
MOD_NVM_RGN           :=  nor
NVM_BASE              :=  0x20100000
NVM_SIZE              :=  0x00080000
endif
ifeq (${PROJ_NVM_REGION},RAM)
MOD_NVM_RGN           :=  ram
NVM_BASE              :=  0x24100000
NVM_SIZE              :=  0x00080000
endif

#
# Configuration of DTCM area.
# In Teseo product, DTCM area usually starts from 0x100000. It is suggested to not
# ITCM area size is affected as well, as total ITCM+DTCM = 256k
#
ifeq (${DATA_TCM_START},)
DATA_TCM_START        :=  0x100000
endif
ifeq (${DATA_TCM_SIZE},)
DATA_TCM_SIZE         :=  0x28000
endif

#
# Configuration of extra HEAP area.
# If size is different from 0, the external heap will be used as standard heap,
# and DTCM is used for fast heap.
# If size equals 0, DTCM is used as standard heap and no fast heap is created.
# NOTE: extra heap base must be a RAM region.
#
ifeq (${OS_HEAP_BASE},)
OS_HEAP_BASE          :=  0x24180000
endif
ifeq (${OS_HEAP_SIZE},)
OS_HEAP_SIZE          :=  0x0
endif


################################################################################
#
# This part should not be modified
#
################################################################################

MOD_CORE              :=  sta8090
