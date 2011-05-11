# Automatic makefile for GNUARM (C/C++)

#Target binary file name
TARGET	   = serial_ex

#Output format (elf,hex,bin)
FORMAT	= hex

#Optimalization [0,1,2,3,s]
# 0 - none optimalization, s - size optimalization 3 - most optimized
OPT 	?= 2

#Define MCU_VARIANT
MCU_VARIANT = f107VBT6


#Common flags 
COMMON_FLAGS = -Wall -DSTM32F10X_CL -pedantic -Wextra

#Include
COMMON_FLAGS += -I../libfoundation/include -I../isix/include -I../isix/arch/arm-cm3/include -I../lib-stm32/inc
LDFLAGS += -Wl,--defsym=_sys_stack_size=256
LDFLAGS += -nodefaultlibs
LDFLAGS += -L../libfoundation -lfoundation -L../lib-stm32/src -lstm32lite -L../isix -lisix


#-lgcc 
#Additional library dependencies
ADDITIONAL_DEPS += ../libfoundation/libfoundation.a ../lib-stm32/src/libstm32lite.a ../isix/libisix.a
	
#CRT0 object with the startup file
CRT0_OBJECT = ../lib-stm32/src/crt0.o

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
SRC += $(wildcard *.c)

#Source C++ files
CPPSRC += $(wildcard *.cpp)

#Source ASM files
ASRC += $(wildcard *.S)


include ../lib-stm32/scripts/stm32.mk

