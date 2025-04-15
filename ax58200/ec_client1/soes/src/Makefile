CROSS = /opt/arm/gcc-arm-none-eabi-10.3-2021.10/bin
CC = $(CROSS)/arm-none-eabi-gcc
CXX = $(CROSS)/arm-none-eabi-g++
OBJCOPY = $(CROSS)/arm-none-eabi-objcopy
OBJDUMP = $(CROSS)/arm-none-eabi-objdump

SRC = \
	main.c \
	system_M480.c \
	AX58200_Hw.c \
	objectlist.c \
	esc.c \
	esc_foe.c \
	esc_eoe.c \
	esc_eep.c \
	esc_hw.c \
	esc_hw_eep.c \
	ecat_slv.c \
	esc_coe.c \
	\
	Library/StdDriver/src/gpio.c \
	Library/StdDriver/src/spi.c \
	Library/StdDriver/src/timer.c \
	Library/StdDriver/src/uart.c

LIB = \
	Library/StdDriver/src/clk.c

HDR = \
	esc.h \
	esc_coe.h \
	esc_foe.h \
	esc_eoe.h \
	esc_eep.h \
	ecat_slv.h \
	options.h

INCS = \
	../../Library/CMSIS/Include \
	../../Library/Device/Nuvoton/M480/Include \
	../../Library/StdDriver/inc

VPATH = \
	../../Library/Device/Nuvoton/M480/Source \
	../../Library/Device/Nuvoton/M480/Source/GCC \
	../../Library/StdDriver/src

#-------------------

OBJ = $(notdir $(SRC:.c=.o) $(LIB:.c=.o))
INC_LIST = $(addprefix -I,$(INCS))

#-------------------

ARCH = -march=armv7 -mtune=cortex-m4 -mthumb

CFLAGS = -g $(INC_LIST) $(ARCH)

LD_FILE = gcc_arm.ld

main.hex: main.elf
	$(OBJCOPY) -O ihex main.elf main.hex

main.elf: $(OBJ) startup_M480.o 
	$(CC) -o main.elf $(ARCH) -T $(LD_FILE) startup_M480.o $(OBJ)
	$(OBJDUMP) -d main.elf >main.elf.dis.txt

startup_M480.o: ../../Library/Device/Nuvoton/M480/Source/GCC/startup_M480.S
	$(CC) -c $(CFLAGS) ../../Library/Device/Nuvoton/M480/Source/GCC/startup_M480.S

clean:
	rm -f *.o *.elf *.hex
