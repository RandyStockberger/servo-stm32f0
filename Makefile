# Compiler & Linker
CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
OC=arm-none-eabi-objcopy
SZ=arm-none-eabi-size

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

# The Nucleo-F072RB board uses an STM32F072RBT6
# The servo board uses an STM32F030K6T6
#
#CDEFS=-DSTM32F072xB
CDEFS=-DSTM32F030x6
#
STARTUP_DEFS=-D__STARTUP_CLEAR_BSS -D__START=main
#
# CMSIS header files are in ./inc
INCPATH=-I./inc
#
CFLAGS=$(ARCH_FLAGS) $(CDEFS) $(INCPATH) -std=c11 -Wall -Os -flto -ffunction-sections -fdata-sections
ASMFLAGS=$(STARTUP_DEFS) $(INCPATH) $(ARCH_FLAGS)

# Link for code size
GC=-Wl,--gc-sections

# Create map file
MAP=-Wl,-Map=servo.map

# Linker script
LDSCRIPTS=-L. -L$(BASE)/ldscripts -T nokeep.ld

LFLAGS=$(USE_NANO) $(USE_NOHOST) $(LDSCRIPTS) $(GC) $(MAP)
#
# --------------------------------------------------
#

servo.bin:	servo.axf
	@echo "Creating servo.bin and servo.srec"
	@$(OC) -O binary servo.axf servo.bin
	@$(OC) -O srec servo.axf servo.srec

servo.elf:	startup_ARMCM0.o system_stm32f0xx.o servo.o led.o pwm.o button.o stm32f0xx_flash.o eeprom.o
	@echo "Creating elf file"
	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@

servo.axf:	startup_ARMCM0.o system_stm32f0xx.o servo.o led.o pwm.o button.o stm32f0xx_flash.o eeprom.o
	@echo "Linking object files"
	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@

pwm.o:	pwm.c pwm.h $(HFILES)
	@echo "Compiling pwm.c"
	@$(CC) -c pwm.c $(CFLAGS)

led.o:	led.c led.h $(HFILES)
	@echo "Compiling led.c"
	@$(CC) -c led.c $(CFLAGS)

button.o:	button.c button.h $(HFILES)
	@echo "Compiling button.c"
	@$(CC) -c button.c $(CFLAGS)

servo.o:	servo.c $(HFILES)
	@echo "Compiling servo.c"
	@$(CC) -c servo.c $(CFLAGS)

eeprom.o:	eeprom.c eeprom.h
	@echo "Compiling eeprom.c"
	@$(CC) -c eeprom.c $(CFLAGS)

stm32f0xx_flash.o:	stm32f0xx_flash.c stm32f0xx_flash.h
	@echo "Compiling stm32f0xx_flash.c"
	@$(CC) -c stm32f0xx_flash.c $(CFLAGS)

system_stm32f0xx.o:	system_stm32f0xx.c $(HFILES)
	@echo "Compiling system_stm32f0xx.c"
	@$(CC) -c system_stm32f0xx.c $(CFLAGS)

startup_ARMCM0.o:	startup_ARMCM0.S
	@echo "Assembling startup_ARMCM0"
	@$(CC) -c startup_ARMCM0.S $(STARTUP_DEFS) $(ASMFLAGS)

prog:	servo.bin
	@echo "Flashing servo.bin to hardware"
	@st-flash --reset write ./servo.bin 0x08000000

#prog:	servo.bin
#	-$(STLINK) -c SWD UR -Q -ME -P servo.bin 0x08000000 -V -Run >prog.txt
#	cat prog.txt

clean: 
	@echo "Cleaning files"
	@rm -f servo.axf *.map *.o servo.elf servo.srec servo.bin prog.txt 

size:
	arm-none-eabi-size servo.axf
