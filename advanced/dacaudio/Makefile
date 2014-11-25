#4 Automatic makefile for GNUARM (C/C++)

#Target binary file name
TARGET	   = dacaudio

#Output format (elf,hex,bin)
FORMAT	= hex

#Optimalization [0,1,2,3,s]
# 0 - none optimalization, s - size optimalization 3 - most optimized
OPT 	?= 2

#MCU variant
MCU_MAJOR_TYPE = f2
MCU_MINOR_TYPE = 05RCT6


#System stack size interrupts etc
MCU_SYSTEM_STACK_SIZE = 512

#Enable exceptions support
CPP_EXCEPTIONS = n

#USB driver type
ISIX_USBDRV_TYPE = stm32f2_host

#Common flags 
COMMON_FLAGS = -Wall -Wextra  -Wno-variadic-macros
#Configuration flags
COMMON_FLAGS += -DFUNCTION_MAIN_RETURN -D_HAVE_CONFIG_H 
COMMON_FLAGS += -I./include

#C compiler options
CFLAGS += $(COMMON_FLAGS)
CFLAGS += -std=gnu99 

#C++ compiler options 
CXXFLAGS += $(COMMON_FLAGS)


#Per file listing
LISTING = n

#Debug version
DEBUG ?=  y


#Source C files
SRC += $(wildcard *.c)

#Source C++ files
CPPSRC += $(wildcard ../../board/*.cpp)
CPPSRC += $(wildcard media/*.cpp)
CPPSRC += $(wildcard *.cpp)

#Source ASM files
ASRC += $(wildcard *.S)

ISIX_BASE_DIR=../../isixrtos
ISIX_DIR=$(ISIX_BASE_DIR)/libisix
ISIXDRV_DIR=$(ISIX_BASE_DIR)/libisixdrvstm32
LIBFOUNDATION_DIR=$(ISIX_BASE_DIR)/libfoundation
LIBSTM32_DIR=$(ISIX_BASE_DIR)/libstm32
LIBGFX_DIR=$(ISIX_BASE_DIR)/libgfx
LIBUSB_DIR=$(ISIX_BASE_DIR)/libusb
LIBFSFAT_DIR=$(ISIX_BASE_DIR)/libfsfat

include $(ISIX_DIR)/isix.mk
include $(LIBSTM32_DIR)/libstm32.mk
include $(ISIXDRV_DIR)/isixdrv.mk
include $(LIBGFX_DIR)/libgfx.mk
include $(LIBUSB_DIR)/libusb.mk
include $(LIBFOUNDATION_DIR)/libfoundation.mk
include $(LIBFSFAT_DIR)/libfsfat.mk
include $(LIBSTM32_DIR)/scripts/stm32.mk
