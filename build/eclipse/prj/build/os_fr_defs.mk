#
# compiler defs for freertos projects
#

OS_ASMINCDIRS:=\
  os/freeRTOS/portable/${TC_ASMTYPE}

OS_CDEFS:=

OS_CINCDIRS:=\
  os                                  \
  os/freeRTOS/portable                \
  os/freeRTOS/Source/include          \
  os/OStoFR/include

OS_LIBNAMES:=\
  os_common                           \
  os_bsp

#
# sources needed for FreeRTOS
#
CSOURCES+=os/freeRTOS/portable/FR_interrupt.c
CSOURCES+=os/freeRTOS/portable/FR_memory.c
CSOURCES+=os/freeRTOS/portable/FR_time.c
CSOURCES+=os/freeRTOS/portable/FR_timelist.c
CSOURCES+=os/freeRTOS/portable/FR_utils.c
CSOURCES+=os/freeRTOS/portable/port.c
CSOURCES+=os/freeRTOS/Source/croutine.c
CSOURCES+=os/freeRTOS/Source/event_groups.c
CSOURCES+=os/freeRTOS/Source/list.c
CSOURCES+=os/freeRTOS/Source/queue.c
CSOURCES+=os/freeRTOS/Source/tasks.c
CSOURCES+=os/freeRTOS/Source/timers.c
CSOURCES+=os/gpOStoFR/gpOStoFR_kernel.c
CSOURCES+=os/gpOStoFR/gpOStoFR_message.c
CSOURCES+=os/gpOStoFR/gpOStoFR_mutex.c
CSOURCES+=os/gpOStoFR/gpOStoFR_semaphore.c
CSOURCES+=os/gpOStoFR/gpOStoFR_task.c
CSOURCES+=os/gpOStoFR/gpOStoFR_utils.c

CARMSOURCES+=os/freeRTOS/portable/portISR.c

ifeq (${PROJ_TC},rvct)
ASMSOURCES+=os/freeRTOS/portable/arm/FR_vectors.s
ASMSOURCES+=os/freeRTOS/portable/arm/interrupti.s
ASMSOURCES+=os/freeRTOS/portable/arm/portasm.s
ASMSOURCES+=os/freeRTOS/portable/arm/stacki.s
ASMSOURCES+=os/freeRTOS/portable/arm/svci.s
endif

ifeq (${PROJ_TC},gae)
ASMSOURCES+=os/freeRTOS/portable/gnu/FR_vectors.asm
ASMSOURCES+=os/freeRTOS/portable/gnu/interrupti.asm
ASMSOURCES+=os/freeRTOS/portable/gnu/stacki.asm
ASMSOURCES+=os/freeRTOS/portable/gnu/svci.asm
endif
