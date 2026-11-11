# make TOOLCHAIN=/caminho/para/tools/bin/arm-none-eabi- BOARD=f411|f767|h723
GIT_BRANCH := $(shell git rev-parse --abbrev-ref HEAD 2>/dev/null)
ifeq ($(GIT_BRANCH),board/nucleo-f767zi)
  DEFAULT_BOARD := f767
else ifeq ($(GIT_BRANCH),board/stm32h723vgt6)
  DEFAULT_BOARD := h723
else
  DEFAULT_BOARD := f411
endif
BOARD ?= $(DEFAULT_BOARD)
IDE_GCC := $(firstword $(wildcard /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/arm-none-eabi-gcc))
TOOLCHAIN ?= $(if $(IDE_GCC),$(patsubst %gcc,%,$(IDE_GCC)),arm-none-eabi-)
CC := $(TOOLCHAIN)gcc
OBJCOPY := $(TOOLCHAIN)objcopy
SIZE := $(TOOLCHAIN)size
TEST_SANITIZERS ?=
BUILD := build
CMSIS ?= Drivers/CMSIS_Core/CMSIS/Core/Include

ifeq ($(BOARD),f767)
  HAL := Drivers/STM32F7xx_HAL_Driver
  DEVICE := Drivers/CMSIS/Device/ST/STM32F7xx
  CPU := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
  MCU_DEF := -DSTM32F767xx
  LD_SCRIPT := STM32F767ZITx_FLASH.ld
  HAL_PREFIX := stm32f7xx_
  HAL_NAMES := hal hal_cortex hal_dma hal_dma_ex hal_exti hal_flash hal_flash_ex hal_gpio hal_i2c hal_i2c_ex hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex
  SYSTEM_SRC := $(DEVICE)/Source/Templates/system_stm32f7xx.c
  STARTUP_ASM := $(DEVICE)/Source/Templates/gcc/startup_stm32f767xx.s
  OPENOCD_TARGET := target/stm32f7x.cfg
else ifeq ($(BOARD),h723)
  HAL := Drivers/STM32H7xx_HAL_Driver
  DEVICE := Drivers/CMSIS/Device/ST/STM32H7xx
  CPU := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
  MCU_DEF := -DSTM32H723xx
  LD_SCRIPT := STM32H723VGTx_FLASH.ld
  HAL_PREFIX := stm32h7xx_
  HAL_NAMES := hal hal_cortex hal_dma hal_dma_ex hal_exti hal_flash hal_flash_ex hal_gpio hal_i2c hal_i2c_ex hal_mdma hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex
  SYSTEM_SRC := $(DEVICE)/Source/Templates/system_stm32h7xx.c
  STARTUP_ASM := $(DEVICE)/Source/Templates/gcc/startup_stm32h723xx.s
  OPENOCD_TARGET := target/stm32h7x.cfg
else
  HAL := Drivers/STM32F4xx_HAL_Driver
  DEVICE := Drivers/CMSIS/Device/ST/STM32F4xx
  CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  MCU_DEF := -DSTM32F411xE
  LD_SCRIPT := STM32F411CEUx_FLASH.ld
  HAL_PREFIX := stm32f4xx_
  HAL_NAMES := hal hal_cortex hal_dma hal_flash hal_flash_ex hal_gpio hal_i2c hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex
  SYSTEM_SRC := $(DEVICE)/Source/Templates/system_stm32f4xx.c
  STARTUP_ASM := $(DEVICE)/Source/Templates/gcc/startup_stm32f411xe.s
  OPENOCD_TARGET := target/stm32f4x.cfg
endif

CPPFLAGS := $(MCU_DEF) -DUSE_HAL_DRIVER -ICore/Inc -I$(HAL)/Inc -I$(DEVICE)/Include -I$(CMSIS)
CFLAGS := $(CPU) -std=c11 -Os -g3 -Wall -Wextra -ffunction-sections -fdata-sections -MMD -MP
LDFLAGS := $(CPU) -T$(LD_SCRIPT) --specs=nano.specs --specs=nosys.specs -u _printf_float -Wl,--gc-sections,-Map=$(BUILD)/firmware.map
SOURCES := $(wildcard Core/Src/*.c) $(addprefix $(HAL)/Src/$(HAL_PREFIX),$(addsuffix .c,$(HAL_NAMES))) $(SYSTEM_SRC)
OBJECTS := $(addprefix $(BUILD)/,$(SOURCES:.c=.o)) $(BUILD)/startup.o

.PHONY: all clean test flash
all: $(BUILD)/firmware.elf $(BUILD)/firmware.bin
$(BUILD)/firmware.elf: $(OBJECTS) $(LD_SCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) -lm -o $@
	$(SIZE) $@
$(BUILD)/firmware.bin: $(BUILD)/firmware.elf
	$(OBJCOPY) -O binary $< $@
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
$(BUILD)/startup.o: $(STARTUP_ASM)
	@mkdir -p $(BUILD)
	$(CC) $(CPU) -c $< -o $@
test:
	@mkdir -p $(BUILD)
	cc -std=c11 -Wall -Wextra -Werror $(TEST_SANITIZERS) -ITests/Stubs -ICore/Inc Tests/test_firmware.c Core/Src/Mpu6050.c Core/Src/Ssd1306.c Core/Src/ComplementaryFilter.c -lm -o $(BUILD)/test_firmware
	./$(BUILD)/test_firmware
flash: all
	openocd -f interface/stlink.cfg -f $(OPENOCD_TARGET) -c "program $(BUILD)/firmware.elf verify reset exit"
clean:
	rm -rf $(BUILD)
-include $(OBJECTS:.o=.d)
