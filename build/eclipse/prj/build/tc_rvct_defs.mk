#
# compiler defs for RVCT toolchain
#

include ../build/macros.mk

################################################################################
#
# This part should be modified only if needed
#
################################################################################

#
# This is the default installation path for DS 5.xx, both x32 and x64.
# Adapt it if the path is different or RVCT 5.xx was installed instead.
#
TC_PATH_INST_DS5:=C:\Program Files\DS-5

#
# This is the default installation path for RVCT 3.1/4.0/4.1.
# Adapt it if the path is different.
#
TC_PATH_INST_RVDS:=C:\Program Files\ARM

################################################################################
#
# This part should not be modified
#
################################################################################

#
# toolchain paths - DS 5.xx related section
#
ifeq ($(findstring DS5,${PROJ_COMPILER}),DS5)
TC_PATH_BIN:=$(call MK_SHORTNAME,"${TC_PATH_INST_DS5}/bin")
TC_PATH_INC:=$(call MK_SHORTNAME,"${TC_PATH_INST_DS5}/include")
TC_PATH_LIB:=$(call MK_SHORTNAME,"${TC_PATH_INST_DS5}/lib")
endif

#
# toolchain paths - RVCT related section
# Up to version 4.1, RVCT is installed in a specific subfolder of RVCT_INST_PATH
#
ifeq ($(findstring RVDS,${PROJ_COMPILER}),RVDS)
ifeq (${PROJ_COMPILER},RVDS31)
RVDS_PATH_VER:=3.1\569
endif
ifeq (${PROJ_COMPILER},RVDS40)
RVDS_PATH_VER:=4.0\650
endif
ifeq (${PROJ_COMPILER},RVDS41)
RVDS_PATH_VER:=4.1\561
endif
TC_PATH_BIN:=$(call MK_SHORTNAME,"${TC_PATH_INST_RVDS}\RVCT\Programs\${RVDS_PATH_VER}\win_32-pentium")
TC_PATH_INC:=$(call MK_SHORTNAME,"${TC_PATH_INST_RVDS}\RVCT\Data\${RVDS_PATH_VER}\include")
TC_PATH_LIB:=$(call MK_SHORTNAME,"${TC_PATH_INST_RVDS}\RVCT\Data\${RVDS_PATH_VER}\lib")
#TC_TC_BINPATH:="${TC_PATH_INST_RVDS};C:\Program Files\ARM\bin\win_32-pentium"
endif

#
# toolchain executables
#
TC_MAKEDEP:=$(call MK_PATHTOUNX,${TC_PATH_BIN}/armcc.exe -M --no_depend_system_headers)

TC_CC:=$(call MK_PATHTOUNX,${TC_PATH_BIN}/armcc.exe)

TC_ASM:=$(call MK_PATHTOUNX,${TC_PATH_BIN}/armasm.exe)

TC_LINK:=$(call MK_PATHTOUNX,${TC_PATH_BIN}/armlink.exe)

TC_GENSCF:=$(call MK_PATHTOUNX,${TC_PATH_BIN}/armcc.exe)

TC_GENBIN:=$(call MK_PATHTOUNX,${TC_PATH_BIN}/fromelf.exe)

#
# toolchain switches macros
#
TC_ASM_VIA=--via ${1}
TC_CC_VIA=--via ${1}
TC_LINK_VIA=--via ${1}
TC_LINK_ENTRY=--entry=${1}
TC_LINK_SCF=--scatter=${1}
TC_LINK_LIST=--list=${1}

#
# constants
#

# toolchain identifiers
TC_NAME:=rvct
TC_SUFFIX:=axf

# Assembly opcodes type
TC_ASMTYPE:=arm

TC_TARGETARM:=--arm
TC_TARGETTHUMB:=--thumb

# Assembly compiler options
TC_ASMOPTS:=\
  --debug                     \
  --diag_style=gnu            \
  --diag_suppress=9931

# Assembly compiler defines
TC_ASMDEFS:=

# C compiler options
TC_COPTS:=\
  -c                          \
  -O2                         \
  -Ospace                     \
  --diag_style=gnu            \
  --diag_suppress=1267,9931   \
  --debug


# C compiler defines
TC_CDEFS:=

# Linker options
TC_LIBOPTS:=\
  --libpath=$(call MK_PATHTOUNX,${TC_PATH_LIB}) \
  --map                                   \
  --verbose                               \
  --info=sizes                            \
  --info=totals                           \
  --info=unused                           \
  --info=stack                            \
  --diag_style=gnu                        \
  --diag_suppress=6314,9931               \
  --symbols

# Linker libraries
TC_LIBNAMES:=
TC_LIBEXT:=ds50.lib

# Scatter file extension
TC_SCFEXT:=scf

#
# toolchain macros
#

# convert PATH to toolchain friendly path
MK_TC_PATH=$(call MK_PATHTOWIN,${1})

# command to generate list of linker directory search paths
MK_TC_LIBDIRS=$(if ${1},--userlibpath $(call MK_SPCTOCOM,${1}))

# command to generate list of linker directory search paths
MK_TC_LIBS=${1}

# command to generate scatter file
# 1 - template file
# 2 - output file
# 3 (opt) - armcc options
MK_TC_GENSCF=$(if $(and ${1},${2},${3}),@${TC_GENSCF} -E ${1} -o ${2} ${3})

# command to generate binary file
# 1 - input file
# 2 - output file
MK_TC_GENBIN=$(if $(and ${1},${2}),@${TC_GENBIN} -c --bin -o ${2} ${1})
