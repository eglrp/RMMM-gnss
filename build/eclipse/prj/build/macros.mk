################################################################################
#
# macros.mk
#
# This makefiles defines all variables and macros used by build makefiles
#
################################################################################

################################################################################
#
# Constants section
#
################################################################################

ifeq (${MACROS_MAKE},)
#$(warning First call)
MACROS_MAKE:=included

# Build process constants
MK_COMMA:= ,
MK_EMPTY:=
MK_SPACE:= ${MK_EMPTY} ${MK_EMPTY}

################################################################################
#
# Macros section
#
################################################################################

# convert path to win path
# 1 - Paths to convert
MK_PATHTOWIN=$(subst /,\,${1})

# convert path to unix path
# 1 - Paths to convert
MK_PATHTOUNX=$(subst \,/,${1})

# Convert spaces to commas
# 1 - string to convert
MK_SPCTOCOM=$(subst ${MK_SPACE},${MK_COMMA},${1})

# Busybox executable
#
MK_BUSYBOX=$(call MK_PATHTOWIN,${PROJ_TOOLSDIR}/busybox.exe)

# Convert non alphanumeric characters to underlines, remove trailing _ and convert to lower case
# 1 - string to convert
MK_FMTNAME=$(shell @${MK_BUSYBOX} echo ${1} | @${MK_BUSYBOX} sed "s/[^a-zA-Z0-9_]/_/g" | @${MK_BUSYBOX} sed "s/_*$$//g" | @${MK_BUSYBOX} tr "[:upper:]" "[:lower:])

# Convert a windows path to 8.3 format
# 1 - string to convert
MK_SHORTNAME=$(shell ${PROJ_TOOLSDIR}/pathtoshort.bat ${1})

# check if file exists
# 1 - File to check
MK_FILEEXIST=$(wildcard $(subst ", ,${1}))

# Echo a string
# 1 - files to type/append
MK_ECHO=$(if ${1},@${MK_BUSYBOX} echo ${1})

# type files/append files to another one
# 1 - files to type/append
# 2 (opt) - destination file
MK_CAT=$(if $(and ${1},${2}),@${MK_BUSYBOX} cat ${1} >> ${2})

# create a folder if it does not exist
# 1 - folder name
MK_MKDIR=$(if ${1},@${MK_BUSYBOX} mkdir -p $(1))

# remove files
# 1 - files to remove
MK_RMFILE=$(if ${1},@${MK_BUSYBOX} rm -f ${1})

# remove directory
# 1 - dir to remove
MK_RMDIR=$(if ${1},@${MK_BUSYBOX} rm -rf ${1})

# copy files
# 1 - source file(s)
# 2 - destination file/folder
MK_CP=$(if $(and ${1},${2}),@${MK_BUSYBOX} cp ${1} ${2})

# append text to a file
# 1 - text to append
# 2 - destination file
MK_APPEND=$(if $(and ${1},${2}),@${MK_BUSYBOX} echo ${1}>>${2})

# convert text to lowercase
# 1 - text to convert
MK_LOWER=$(if ${1},$(shell @${MK_BUSYBOX} echo ${1} | ${MK_BUSYBOX} tr '[:upper:]' '[:lower:]' ))

# convert text to uppercase
# 1 - text to convert
MK_UPPER=$(if ${1},$(shell @${MK_BUSYBOX} echo ${1} | ${MK_BUSYBOX} tr '[:lower:]' '[:upper:]' ))

# configure a binary
# 1 - source file
# 2 - destination file
# 3 - configuration file
MK_FWCONFIG=$(if $(and ${1},${2},${3}),@"${PROJ_TOOLSDIR}/FWConfig" -f ${1} -c ${3} -o ${2})

# configure a binary
# 1 - source file
# 2 - destination file
# 3 - configuration file
MK_FWCHECKCRC=$(if ${1},@"${PROJ_TOOLSDIR}/FWConfig" -f ${1} -crc)

# flash a binary image using XLoader protocol
# 1 - source binary image
# 2 - com port
# 3 - com speed
# 4 - load region
# 5 - start address
MK_FWPROGRAM=${PROJ_TOOLSDIR}/TeseoProgrammer.exe program -f t3 -v sta8089fg -c ${2} -b ${3} -m ${4} -d ${5}\
  $(if $(findstring on,${PROJ_BOARD_ERASENVM}),-e true)\
  -i $(call MK_PATHTOWIN,${1})

# upgrade a binary image using FWUpgrade protocol
# 1 - source binary image
# 2 - com port
# 3 - com speed
MK_FWUPGRADE=${PROJ_TOOLSDIR}/TeseoProgrammer.exe upgrade -c ${2} -b ${3}\
  $(if $(findstring on,${PROJ_BOARD_ERASENVM}),-e true)\
  $(if $(findstring on,${PROJ_BOARD_UPGRECOVERY}),-r true)\
  -i $(call MK_PATHTOWIN,${1})
else
#$(warning Nested call)
endif

