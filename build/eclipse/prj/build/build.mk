################################################################################
#
# build.mk
#
# Build rules to generate exe file from sources
#
################################################################################

include ../build/const.mk

include ../build/core_defs.mk

include ../build/check.mk

include ../build/common.mk

include ../build/dbg_ozone.mk

################################################################################
#
# Constants section
#
################################################################################

################################################################################
#
# Macros section
#
################################################################################

# Filter a source file from a list.
# As vpath is used to add c/s/asm sources paths to make search folders, it could
# happen that a file with same name is present in more folders. To avoid
# building the wrong one, this macro can filter a sources list and return the
# proper file in the list.
# 1 - source to look for (with or without path)
# 2 - target sources lists
MK_LOOKUPSRC=$(filter %/$(notdir ${1}),${2})

################################################################################
#
# Variable section
#
################################################################################

# C thumb sources
BUILD_CSOURCES:=$(foreach src,${PROJ_CSOURCES},${PROJ_SDKROOTDIR}/$(src))

# C ARM sources
BUILD_CARMSOURCES:=$(foreach src,${PROJ_CARMSOURCES},${PROJ_SDKROOTDIR}/$(src))

# Assembly sources
BUILD_ASMSOURCES:=$(foreach src,${PROJ_ASMSOURCES},${PROJ_SDKROOTDIR}/$(src))

# All sources
BUILD_SOURCES:=${BUILD_CSOURCES} ${BUILD_CARMSOURCES} ${BUILD_ASMSOURCES}

# All dependency files
BUILD_DEPENDS:=$(patsubst %.c,%.d,${BUILD_CSOURCES} ${BUILD_CARMSOURCES})

# All sources
PROJ_OBJS:=$(foreach obj,$(basename $(notdir ${BUILD_SOURCES})),${OBJS_FOLDER}/${obj}.o)

# Add sources paths to makefile search paths specific for each pattern
vpath %.c $(dir ${BUILD_CSOURCES} ${BUILD_CARMSOURCES})
vpath %.s $(dir ${BUILD_ASMSOURCES})
vpath %.asm $(dir ${BUILD_ASMSOURCES})

################################################################################
#
# Targets section
#
################################################################################

#
# Folders targets
#
${TARGET_ROOTDIR}:
	$(call MK_MKDIR,"$(call MK_PATHTOWIN,$@)")

${OBJS_FOLDER}:
	$(call MK_MKDIR,"$(call MK_PATHTOWIN,$@)")

${COMMON_BINDIR}:
	$(call MK_MKDIR,"$(call MK_PATHTOWIN,$@)")

#
# Via files targets
#
${SCF_FILE_NAME}: ${CONFIG_FILE} ${SCF_TEMPLATE_FILE_NAME} ${DEFSINCLIST}
	$(call MK_ECHO,Generating scatter file for ${TC_RVCT_NAME} builder)
	$(call MK_RMFILE,${SCF_FILE_NAME})
	$(call MK_TC_GENSCF,"${SCF_TEMPLATE_FILE_NAME}","${SCF_FILE_NAME}",${SCF_DEFS})

${ASMOPTS_FILE}: ${CONFIG_FILE} ${DEFSINCLIST} ${PROJ_MAKEFILE}
	$(call MK_ECHO,Generating assembler via file for ${TC_RVCT_NAME} builder)
	$(call MK_RMFILE,${ASMOPTS_FILE})
	$(call MK_APPEND,${ASMOPTS},"${ASMOPTS_FILE}")
	$(call MK_APPEND,${ASMDEFS},"${ASMOPTS_FILE}")
	$(call MK_APPEND,${ASMINCDIRS},"${ASMOPTS_FILE}")

${COPTS_FILE}: ${CONFIG_FILE} ${DEFSINCLIST} ${PROJ_MAKEFILE}
	$(call MK_ECHO,Generating C compiler via file for ${TC_RVCT_NAME} builder)
	$(call MK_RMFILE,${COPTS_FILE})
	$(call MK_APPEND,${COPTS},"${COPTS_FILE}")
	$(call MK_APPEND,${CDEFS},"${COPTS_FILE}")
	$(call MK_APPEND,${CINCDIRS},"${COPTS_FILE}")

${LOPTS_FILE}: ${CONFIG_FILE} ${DEFSINCLIST} ${PROJ_MAKEFILE}
	$(call MK_ECHO,Generating linker via file for ${TC_RVCT_NAME} builder)
	$(call MK_RMFILE,${LOPTS_FILE})
	$(call MK_APPEND,${LIBOPTS},"${LOPTS_FILE}")
	$(call MK_APPEND,$(call MK_TC_LIBDIRS,${LIBDIRS}),"${LOPTS_FILE}")
	$(call MK_APPEND,$(call MK_TC_LIBS,${LIBS}),"${LOPTS_FILE}")

#
# FW config target
#
${FWCFG_FILE}: ${DEF_FWCFG_FILE} configs/${PROJ_CFG}.mk $(if $(call MK_FILEEXIST,"${PROJ_FWCFG_FILE}"),${PROJ_FWCFG_FILE})
	$(call MK_ECHO,Generating config file)
	$(call MK_RMFILE,${FWCFG_FILE})
	$(call MK_CAT,"${DEF_FWCFG_FILE}","${FWCFG_FILE}")
	$(if $(call MK_FILEEXIST,"${PROJ_FWCFG_FILE}"),$(call MK_CAT,"${PROJ_FWCFG_FILE}","${FWCFG_FILE}"))
	$(call MK_APPEND,${FWCFGTCXO},"${FWCFG_FILE}")

#
# Compiler implicit targets
#
#${OBJS_FOLDER}/%.o: %.c ${OBJS_FOLDER}/%.d
${OBJS_FOLDER}/%.o: %.c ${COPTS_FILE}
	$(eval SRCFILE:=$(call MK_LOOKUPSRC,$^,${BUILD_CSOURCES} ${BUILD_CARMSOURCES}))
	$(eval SRCMODE:=$(if $(findstring $<,${BUILD_CARMSOURCES}),${TC_TARGETARM},${TC_TARGETTHUMB}))
	$(call MK_ECHO,Compiling ${SRCFILE})
	@${TC_CC} $(call TC_CC_VIA,${TARGET_ROOTDIR}/copts.via) ${SRCMODE} \
	  -o $@ \
	     ${SRCFILE}

${OBJS_FOLDER}/%.o: %.s ${ASMOPTS_FILE}
	$(eval SRCFILE:=$(call MK_LOOKUPSRC,$^,${BUILD_ASMSOURCES}))
	$(call MK_ECHO,Assembling ${SRCFILE})
	@${TC_ASM} $(call TC_ASM_VIA,${TARGET_ROOTDIR}/asmopts.via) \
	  -o $@ \
	     ${SRCFILE}

${OBJS_FOLDER}/%.o: %.asm ${ASMOPTS_FILE}
	$(eval SRCFILE:=$(call MK_LOOKUPSRC,$^,${BUILD_ASMSOURCES}))
	$(call MK_ECHO,Assembling ${SRCFILE})
	@${TC_ASM} $(call TC_ASM_VIA,${TARGET_ROOTDIR}/asmopts.via) \
	  -o $@ \
	     ${SRCFILE}

#.PRECIOUS: ${OBJS_FOLDER}/%.d
#${OBJS_FOLDER}/%.d: %.c
#	$(eval SRCFILE:=$(call MK_LOOKUPSRC,$^,${BUILD_CSOURCES} ${BUILD_CARMSOURCES}))
#	$(call MK_ECHO,Building dependency for ${SRCFILE})
#	@${TC_MAKEDEP} $(call TC_CC_VIA,${TARGET_ROOTDIR}/copts.via) \
#		-o $(patsubst %.d,%.o,$@) ${SRCFILE} > $@
#		-o $(patsubst %.d,%.o,$@) ${SRCFILE} > $@
#		${SRCFILE} > $@_tmp ; \
#	  $(MK_BUSYBOX) awk "{ print \"pippo/\"$1\" \"$2}" $@_tmp)
#	$(call MK_RMFILE,$@_tmp)

-include ${OBJS_FOLDER}/%.d

#
# Executable target
#
${TARGET_SRC_FILENAME_EXE}: ${LOPTS_FILE} ${SCF_FILE_NAME} ${OBJS_FOLDER} ${PROJ_OBJS} ${LOPTS_FILE}
	$(call MK_ECHO,Linking to $(subst ${PROJ_SDKROOTDIR}/,,$@))
	@${TC_LINK} -o $@ ${PROJ_OBJS} \
	  $(call TC_LINK_VIA,${TARGET_ROOTDIR}/lopts.via) \
	  $(call TC_LINK_ENTRY,${PROJ_ENTRY_POINT}) \
	  $(call TC_LINK_SCF,${SCF_FILE_NAME}) \
	  $(call TC_LINK_LIST,${TARGET_SRC_FILENAME_LIST})
#	$(if $(findstring ${MOD_APP_FLDNAME},gpsapp),@${MAKE} applydefcfg)

#
# Apply FW configuration for GNSS targets
#
.SECONDARY: applydefcfg
applydefcfg: ${TARGET_SRC_FILENAME_EXE}
	$(call MK_ECHO,Applying default config)
	$(call MK_CP,"${TARGET_SRC_FILENAME_EXE}","${TARGET_SRC_FILENAME_EXE}_temp")
	$(call MK_FWCONFIG,"${TARGET_SRC_FILENAME_EXE}_temp","${TARGET_SRC_FILENAME_EXE}","${FWCFG_FILE}")
	$(call MK_RMFILE,"${TARGET_SRC_FILENAME_EXE}_temp")

#
# Binary target
#
${TARGET_DEST_FILENAME_BIN}: ${COMMON_BINDIR} ${TARGET_SRC_FILENAME_EXE} ${TARGET_EXTRA_RULES}
	$(call MK_ECHO,Copying to bin folder)
	$(call MK_RMFILE,"${TARGET_DEST_FILENAME_EXE}" "${TARGET_DEST_FILENAME_LIST}" "${TARGET_DEST_FILENAME_BIN}")
	$(call MK_CP,"${TARGET_SRC_FILENAME_EXE}","${TARGET_DEST_FILENAME_EXE}")
	$(call MK_CP,"${TARGET_SRC_FILENAME_LIST}","${TARGET_DEST_FILENAME_LIST}")
	$(call MK_ECHO,Generating standard binary image)
	$(call MK_TC_GENBIN,"${TARGET_SRC_FILENAME_EXE}","${TARGET_DEST_FILENAME_BIN}")

#
# FW upgrade target
#
${TARGET_DEST_FILENAME_BOOT}: ${TARGET_DEST_FILENAME_BIN}
	$(call MK_ECHO,Generating boot and upgrade binaries)
	$(call MK_RMFILE,"${TARGET_DEST_FILENAME_UPG}" "${TARGET_DEST_FILENAME_BOOT}")
	$(call MK_CP,"${TARGET_DEST_FILENAME_BIN}","${TARGET_DEST_FILENAME_UPG}")
	$(call MK_CAT,"${SECLOADER_IMAGE}" "${SECLOADER_CODEVAL}" "${TARGET_DEST_FILENAME_BIN}","${TARGET_DEST_FILENAME_BOOT}")
	$(call MK_RMFILE,"${TARGET_DEST_FILENAME_BIN}")
	$(call MK_FWCHECKCRC,"${TARGET_DEST_FILENAME_BOOT}")

#
# Ozone debugger project
#
${OZONE_PROJ}: ${PROJ_DEBUGDIR}/${OZONE_PROJ_TEMPLATE} ${TARGET_DEST_FILENAME_BIN}
	$(call MK_ECHO,Generating Ozone script for ${PROJ_CFG})
	@${MK_BUSYBOX} cat "${PROJ_DEBUGDIR}/${OZONE_PROJ_TEMPLATE}" | \
	  ${MK_BUSYBOX} sed "s/__PROJROOTDIR__/$(subst /,\/,${PROJ_SDKROOTDIR})/g" | \
	  ${MK_BUSYBOX} sed "s/__IMAGENAME__/$(subst /,\/,${TARGET_DEST_FILENAME_EXE})/g" \
	  > "${OZONE_PROJ}"

#
# internal secondary targets
#
.SECONDARY: generatemk build postbuild clean distclean program debug

FORCE:

# pre build target
prebuild: ${TARGET_ROOTDIR} ${SCF_FILE_NAME} ${ASMOPTS_FILE} ${COPTS_FILE} ${LOPTS_FILE} ${FWCFG_FILE}

# build makefile target
build: ${TARGET_SRC_FILENAME_EXE}

# apply default configuration target
postbuild: ${TARGET_POSTBUILD} ${OZONE_PROJ}

#
# exported targets
#
clean:
	$(call MK_ECHO,Executing clean on target ${PROJ_CFG})
	$(call MK_RMFILE,${PROJ_OBJS})
	$(call MK_RMFILE,${TARGET_SRC_FILENAME_EXE})
	$(call MK_RMFILE,${TARGET_SRC_FILENAME_LIST})

distclean:
	$(call MK_ECHO,Executing distclean on target ${PROJ_CFG})
	$(call MK_RMDIR,${TARGET_ROOTDIR})

program: ${TARGET_POSTBUILD}
	$(call MK_ECHO,Programming image)
	$(call MK_FWPROGRAM,${TARGET_POSTBUILD},${PROJ_BOARD_COM},${PROJ_BOARD_COMSPEED},$(call MK_UPPER,${MOD_LOAD_RGN}),${LR_CODE_BASE})

upgrade: ${TARGET_POSTBUILD}
ifeq (${MOD_FWUPG},1)
	$(call MK_ECHO,Upgrading image)
	$(call MK_FWUPGRADE,${TARGET_DEST_FILENAME_UPG},${PROJ_BOARD_COM},${PROJ_BOARD_COMSPEED})
else
	$(call MK_ECHO,Upgrading off for this target)
endif

debug: ${OZONE_PROJ}
	$(call MK_ECHO,Starting Ozone debugger)
	${DBG_OZONE_EXE} ${OZONE_PROJ}

rebuild:
	${MAKE} clean 
	${MAKE} all

regenerate:
	${MAKE} distclean 
	${MAKE} all