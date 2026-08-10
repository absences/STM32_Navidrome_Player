CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

ROOT = ..
BUILD = build
TARGET = music_controller
CPU = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard
CFLAGS = $(CPU) -DSTM32H743xx -O2 -g -Wall -Wextra -ffunction-sections -fdata-sections \
	-I$(ROOT)/Drivers/CMSIS/Device/ST/STM32H7xx/Include -I$(ROOT)/Drivers/CMSIS/Include \
	-IThirdParty/minimp3
LDFLAGS = $(CPU) -T$(ROOT)/STM32H743IITX_FLASH.ld -Wl,-gc-sections --specs=nosys.specs
SOURCES = Src/main.c \
	Src/sdram.c \
	Src/wm8978_playback.c \
	Src/minimp3_impl.c \
	$(ROOT)/Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c \
	$(ROOT)/Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h743xx.s
OBJECTS = $(addprefix $(BUILD)/,$(addsuffix .o,$(basename $(SOURCES))))

all: $(BUILD)/$(TARGET).hex

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@
	$(OBJCOPY) -O binary $< $(BUILD)/$(TARGET).bin

clean:
	rm -rf $(BUILD)

.PHONY: all clean
