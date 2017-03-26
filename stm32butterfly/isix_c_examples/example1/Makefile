#4 Automatic makefile for GNUARM (C/C++)

#Target binary file name
TARGET	   = example1c

#Output format (elf,hex,bin)
FORMAT	= elf

#Optimalization [0,1,2,3,s]
# 0 - none optimalization, s - size optimalization 3 - most optimized
OPT 	?= 2

#MCU variant
MCU_MAJOR_TYPE = f1
MCU_MINOR_TYPE = 07VBT6

#Enable exceptions support
CPP_EXCEPTIONS = n

#Common flags 
COMMON_FLAGS = -Wall -DSTM32F10X_CL -Wextra  -Wno-variadic-macros  -fshort-enums

#Include
COMMON_FLAGS += -DCOMPILED_UNDER_ISIX -DISIX_CONFIG_STACK_GROWTH
COMMON_FLAGS +=  $(LIBFOUNDATION_INC) $(ISIX_INC) $(ISIXDRV_INC) $(LIBSTM32_INC)
LDFLAGS += -Wl,--defsym=_sys_stack_size=384


#C compiler options
CFLAGS += $(COMMON_FLAGS)

#C++ compiler options 
CXXFLAGS += $(COMMON_FLAGS)

#Disable safe constructors
#CXXFLAGS +=  -fno-threadsafe-statics

#Per file listing
LISTING = n

#Debug version
DEBUG ?=  y


#Source C files
SRC += $(wildcard *.c) 

#Source C++ files
CPPSRC += $(wildcard *.cpp)

#Source ASM files
ASRC += $(wildcard *.S)

ISIX_DIR=../../isixrtos/libisix
ISIX_ARCH=arm-cm3
ISIXDRV_DIR=../../isixrtos/libisixdrvstm32
LIBFOUNDATION_DIR=../../isixrtos/libfoundation
LIBSTM32_DIR=../../isixrtos/libstm32
include $(ISIX_DIR)/isix.mk
include $(ISIXDRV_DIR)/isixdrv.mk
include $(LIBFOUNDATION_DIR)/libfoundation.mk
include $(LIBSTM32_DIR)/libstm32.mk
include $(LIBSTM32_DIR)/scripts/stm32.mk

