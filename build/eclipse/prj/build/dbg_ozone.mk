#
# debugger defs for Ozone
#

################################################################################
#
# This part should be modified only if needed
#
################################################################################

#
# This is the default installation path for Ozone
#
DBG_OZONE_PATH_INST:=C:\Program Files (x86)\Segger\Ozone V2.22b

################################################################################
#
# This part should not be modified
#
################################################################################

DBG_OZONE_EXE:=@${PROJ_TOOLSDIR}/start_ozone.bat $(call MK_PATHTOUNX,$(call MK_SHORTNAME,"${DBG_OZONE_PATH_INST}"))/Ozone.exe

