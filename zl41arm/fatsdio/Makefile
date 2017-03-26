#4 Automatic makefile for GNUARM (C/C++)

#Target binary file name
TARGET	   = fatsdio

#Output format (elf,hex,bin)
FORMAT	= elf

#Optimalization [0,1,2,3,s]
# 0 - none optimalization, s - size optimalization 3 - most optimized
OPT 	?= 2

#MCU variant
MCU_MAJOR_TYPE = f4
MCU_MINOR_TYPE = 17VGT6


#System stack size interrupts etc
MCU_SYSTEM_STACK_SIZE = 512

#Enable exceptions support
CPP_EXCEPTIONS = y

#Common flags 
COMMON_FLAGS = -Wall -Wextra -Wno-variadic-macros

#C compiler options
CFLAGS += $(COMMON_FLAGS)

#C++ compiler options 
CXXFLAGS += $(COMMON_FLAGS) --std=c++11


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

ISIX_BASE_DIR=../../isixrtos
ISIX_DIR=$(ISIX_BASE_DIR)/libisix
LIBFSFAT_DIR=$(ISIX_BASE_DIR)/libfsfat
ISIXDRV_DIR=$(ISIX_BASE_DIR)/libisixdrvstm32
LIBFOUNDATION_DIR=$(ISIX_BASE_DIR)/libfoundation
LIBSTM32_DIR=$(ISIX_BASE_DIR)/libstm32

include $(LIBFSFAT_DIR)/libfsfat.mk
include $(ISIXDRV_DIR)/isixdrv.mk
include $(LIBFOUNDATION_DIR)/libfoundation.mk
include $(LIBSTM32_DIR)/libstm32.mk
include $(ISIX_DIR)/isix.mk
include $(LIBSTM32_DIR)/scripts/stm32.mk

