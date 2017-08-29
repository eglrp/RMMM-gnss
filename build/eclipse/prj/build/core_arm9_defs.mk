################################################################################
#
# ARM946 configuration file
#
################################################################################

ifeq (${TC_NAME},rvct)
CORE_ASMOPTS:=\
  --cpu=ARM946E-S                   \
  --fpu=SoftVFP                     \
  --apcs=/interwork

CORE_COPTS:=\
  --cpu=ARM946E-S                   \
  --fpu=SoftVFP                     \
  --apcs=/interwork/noropi/norwpi   \
  --no_unaligned_access
endif # TC_RVCT_NAME

ifeq (${TC_NAME},gae)
CORE_ASMOPTS:=\
  -mcpu=arm946e-s                   \
  -mtune=arm946e-s                  \
  -march=armv5te                    \
  -mfloat-abi=soft                  \
  -mthumb-interwork

CORE_COPTS:=\
  -mcpu=arm946e-s                   \
  -mtune=arm946e-s                  \
  -march=armv5te                    \
  -mfloat-abi=soft                  \
  -mthumb-interwork                 \
  -fomit-frame-pointer              \
  -fno-exceptions                   \
  -ffast-math                       \
  -fno-common                       \
  -funroll-loops                    \
  -mlong-calls                      \
  -mlittle-endian                   \
  -ffunction-sections               \
  -fdata-sections
endif # TC_GCC_NAME
