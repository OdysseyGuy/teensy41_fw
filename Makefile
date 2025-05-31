MCU = IMXRT1062
MCU_LD = imxrt1062.ld

TARGET = main

OPTIONS += -D__$(MCU)__
CPUOPTIONS = -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb

COMPILERPATH = $(abspath ../x86_64-arm-none-eabi/bin)

CFLAGS = -Wall -g -O0 $(CPUOPTIONS) -MMD $(OPTIONS) -I. -ffunction-sections -fdata-sections
LDFLAGS = -Os -Wl,--gc-sections,--relax $(SPECS) $(CPUOPTIONS) -T$(MCU_LD)

CC = $(COMPILERPATH)/arm-none-eabi-gcc
CXX = $(COMPILERPATH)/arm-none-eabi-g++
OBJCOPY = $(COMPILERPATH)/arm-none-eabi-objcopy
SIZE = $(COMPILERPATH)/arm-none-eabi-size

C_FILES := $(wildcard *.c)
OBJS := $(C_FILES:.c=.o)

all: $(TARGET).hex

$(TARGET).elf: $(OBJS) $(MCU_LD)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@

-include $(OBJS:.o=.d)

clean:
	rm -r *.o *.d $(TARGET).elf $(TARGET).hex
