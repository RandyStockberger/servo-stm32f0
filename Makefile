# Compiler & Linker
CC=\ARMGCC\bin\arm-none-eabi-gcc
AS=\ARMGCC\bin\arm-none-eabi-as
CXX=\ARMGCC\bin\arm-none-eabi-g++
OC=\ARMGCC\bin\arm-none-eabi-objcopy
SZ=arm-none-eabi-size
STLINK="C:/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility/ST-LINK_CLI.exe"
#
CMSISLIB=C:/Users/randy/src/Embedded/ARM-STM/Downloads/stm32f0_stdperiph_lib/STM32F0xx_StdPeriph_Lib_V1.5.0/

NAME=servo

HFILES=inc/stm32f0xx.h inc/system_stm32f0xx.h

# Use newlib-nano. To disable it, specify USE_NANO=
USE_NANO=--specs=nano.specs

# Use seimhosting or not
USE_SEMIHOST=--specs=rdimon.specs
USE_NOHOST=--specs=nosys.specs

# Startup code
STARTUP=startup_ARMCM0

# Options for specific architecture
ARCH_FLAGS=-mthumb -mcpu=cortex-m0

# CDEFS=-DSTM32F030 -DSTM32F030x6
CDEFS=-DSTM32F072 -DSTM32F072xB
STARTUP_DEFS=-D__STARTUP_CLEAR_BSS -D__START=main

# CMSIS header files are in "../../../Libraries/CMSIS/Include"
#ISEARCH= -I $(CMSISLIB)Libraries/CMSIS/Include
ISEARCH=-I./inc

# -Os -flto -ffunction-sections -fdata-sections to compile for code size
#CFLAGS=$(ARCH_FLAGS) $(STARTUP_DEFS) $(ISEARCH) -Os -flto -ffunction-sections -fdata-sections
CFLAGS=$(ARCH_FLAGS) $(CDEFS) $(ISEARCH) -std=c11 -Os -flto -ffunction-sections -fdata-sections
CXXFLAGS=$(CFLAGS)
AFLAGS=$(STARTUP_DEFS) $(ISEARCH) $(ARCH_FLAGS)

# Link for code size
GC=-Wl,--gc-sections

# Create map file
MAP=-Wl,-Map=$(NAME).map

# Linker script
LDSCRIPTS=-L. -L$(BASE)/ldscripts -T nokeep.ld

LFLAGS=$(USE_NANO) $(USE_NOHOST) $(LDSCRIPTS) $(GC) $(MAP)
#
# --------------------------------------------------
#

$(NAME).bin:	$(NAME).axf
	$(OC) -O binary $(NAME).axf $(NAME).bin
	$(OC) -O srec $(NAME).axf $(NAME).srec

$(NAME).axf:	$(NAME).o $(STARTUP).o system_stm32f0xx.o
	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@

$(NAME).o:	$(NAME).c $(HFILES)
	$(CC) -c $(NAME).c $(CFLAGS)

system_stm32f0xx.o:	system_stm32f0xx.c inc/stm32f0xx.h
	$(CC) -c system_stm32f0xx.c $(CFLAGS)

$(STARTUP).o:	$(STARTUP).S
	$(CC) -c $^ $(AFLAGS)

prog:	$(NAME).bin
	$(STLINK) -c SWD UR -Q -ME -P $(NAME).bin 0x08000000 -V -Run >out.txt
	@cat out.txt
	@rm out.txt

clean: 
	rm -f $(NAME).axf *.map *.o $(NAME).srec $(NAME).bin out.txt

