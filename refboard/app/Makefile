#4 Automatic makefile for GNUARM (C/C++)

#Target binary file name
TARGET	   = btnxclock

#Output format (elf,hex,bin)
FORMAT	= hex


#Optimalization [0,1,2,3,s]
OPT 	?= s

#MCU variant
MCU_MAJOR_TYPE = f4
MCU_MINOR_TYPE = 07ZET6

#Enable bootloader code
USE_SBL_BOOTLOADER = n
#System stack size interrupts etc
MCU_SYSTEM_STACK_SIZE = 512
#Enable exceptions support
CPP_EXCEPTIONS = n
#Per file listing
LISTING = n
#Debug version
DEBUG ?=  y
#Programer configuration
PGM_SWD = y
PGM_TYPE=stlink-v2-1


#Common flags 
COMMON_FLAGS = -Wall -Wextra  -Wno-variadic-macros
#Configuration flags
COMMON_FLAGS += -DFUNCTION_MAIN_RETURN -D_HAVE_CONFIG_H
COMMON_FLAGS += -I../board/include

#C compiler options
CFLAGS += $(COMMON_FLAGS)

#C++ compiler options 
CXXFLAGS += $(COMMON_FLAGS)


#Source C files
SRC += $(wildcard *.c)

#Source C++ files
CPPSRC += $(wildcard ../board/*.cpp)
CPPSRC += $(wildcard *.cpp)

#Source ASM files
ASRC += $(wildcard *.S)


ISIX_BASE_DIR=../../isixrtos
ISIX_DIR=$(ISIX_BASE_DIR)/libisix
ISIXDRV_DIR=$(ISIX_BASE_DIR)/libisixdrvstm32
LIBFOUNDATION_DIR=$(ISIX_BASE_DIR)/libfoundation
LIBGFX_DIR=$(ISIX_BASE_DIR)/libgfx
LIBSTM32_DIR=$(ISIX_BASE_DIR)/libstm32

include $(ISIXDRV_DIR)/isixdrv.mk
include $(LIBSTM32_DIR)/libstm32.mk
include $(LIBGFX_DIR)/libgfx.mk
include $(LIBFOUNDATION_DIR)/libfoundation.mk
include $(ISIX_DIR)/isix.mk
include $(LIBSTM32_DIR)/scripts/stm32.mk
