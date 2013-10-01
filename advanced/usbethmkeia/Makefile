#4 Automatic makefile for GNUARM (C/C++)

#Target binary file name
TARGET	   = usbtcpipdemo

#Output format (elf,hex,bin)
FORMAT	= elf

#Optimalization [0,1,2,3,s]
# 0 - none optimalization, s - size optimalization 3 - most optimized
OPT 	?= 2


#MCU variant
MCU_MAJOR_TYPE = f1
MCU_MINOR_TYPE = 07VCT6


#System stack size interrupts etc
MCU_SYSTEM_STACK_SIZE = 512


#Enable exceptions support
CPP_EXCEPTIONS = n

#Common flags
COMMON_FLAGS = -Iinclude -D_HAVE_CONFIG_H
COMMON_FLAGS += -Wall -DSTM32F10X_CL -pedantic -Wextra  -Wno-variadic-macros

#Include
COMMON_FLAGS += -DCOMPILED_UNDER_ISIX -DISIX_CONFIG_STACK_GROWTH
COMMON_FLAGS +=  $(LIBFOUNDATION_INC) $(ISIX_INC) $(ISIXDRV_INC) $(LIBSTM32_INC)


#C compiler options
CFLAGS += $(COMMON_FLAGS)
CFLAGS += -std=gnu99 

#C++ compiler options 
CXXFLAGS += $(COMMON_FLAGS)

#Disable safe constructors
#CXXFLAGS +=  -fno-threadsafe-statics

#Per file listing
LISTING = n

#Debug version
DEBUG ?=  y

#Source C files
SRC += $(wildcard *.c) tcpecho/tcpecho.c

#Source C++ files
CPPSRC += $(wildcard *.cpp) 

#Source ASM files
ASRC += $(wildcard *.S)

#Isix base directory
ISIX_PROJECT_DIR 	:=	../../isixrtos
#Isix architecture
ISIX_ARCH			:=	arm-cm3

#Target libraries
ISIX_DIR			:=	$(ISIX_PROJECT_DIR)/libisix
ISIXDRV_DIR			:=	$(ISIX_PROJECT_DIR)/libisixdrvstm32
LIBFOUNDATION_DIR	:=  $(ISIX_PROJECT_DIR)/libfoundation
LIBSTM32_DIR		:=	$(ISIX_PROJECT_DIR)/libstm32
LIBTCPIP_DIR		:=	$(ISIX_PROJECT_DIR)/libtcpip
LIBUSBSTM32_DIR		:=	$(ISIX_PROJECT_DIR)/libusbstm32


LIBUSBSTM32_NO_HOST = y


include $(LIBTCPIP_DIR)/libtcpip.mk
include $(LIBUSBSTM32_DIR)/libusbstm32.mk
include $(ISIX_DIR)/isix.mk
include $(ISIXDRV_DIR)/isixdrv.mk
include $(LIBFOUNDATION_DIR)/libfoundation.mk
include $(LIBSTM32_DIR)/libstm32.mk
include $(LIBSTM32_DIR)/scripts/stm32.mk

