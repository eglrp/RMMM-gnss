################################################################################
#
# Configurable section
#
# This section can be configured by customer to add more options to build process:
# - EXT_CSOURCES can be used to add custom C source files
# - EXT_ASMSOURCES can be used to add custom assembler source files
# - EXT_ASMDEFS can be used to add custom assembler defines
# - EXT_ASMINCDIRS can be used add custom folders to be searched for assembler includes
# - EXT_ASMOPTS can be used to add custom assembler options
# - EXT_CDEFS can be used to add custom C defines
# - EXT_CINCDIRS can be used add custom folders to be searched for C includes
# - EXT_COPTS can be used to add custom C options
# - EXT_LIBS can be used to add custom libs to be linked
# - EXT_LIBDIRS can be used to add custom folders to be searched for libs
# - EXT_LIBOPTS can be used to add custom lib options
#
################################################################################

#
# Assembler extra options
#

# extra C sources that will be built in thumb mode
EXT_CSOURCES=

# extra C sources that will be built in ARM mode
EXT_CARMSOURCES=

# extra assembly sources
EXT_ASMSOURCES=

#
# Assembler extra options
#
EXT_ASMDEFS=

EXT_ASMINCDIRS=

EXT_ASMOPTS=

#
# C compiler extra options
#
EXT_CDEFS=

EXT_CINCDIRS=

EXT_COPTS=

#
# C linker extra options
#
EXT_LIBS=

EXT_LIBDIRS=

EXT_LIBOPTS=

################################################################################
#
# End of configurable section
#
################################################################################

#
# including defines related to configuration
#
include configs/${PROJ_CFG}.mk

#
# including programming and debugging options
#
include prgdbg.mk

#
# including sources file list
#
include files.mk

#
# including extra options
#
-include extras.mk

include ../build/build.mk

all: prebuild build postbuild
