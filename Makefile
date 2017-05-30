GNUMAKEFLAGS = --no-print-directory

CROSS_COMPILE = arm-none-eabi-
SRC_ROOT = ./Src
LIB_ROOT = ./Lib

CHIP = STM32F411xE
BOARD = USE_STM32F4XX_NUCLEO
LD_SCRIPT = $(LIB_ROOT)/Projects/STM32F411RE-Nucleo/Templates/TrueSTUDIO/STM32F4xx-Nucleo/STM32F411RE_FLASH.ld


CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
AS = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
RANLIB = $(CROSS_COMPILE)ranlib
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS = -mcpu=cortex-m4 -mthumb
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -Wall -Wextra -fsigned-char
CFLAGS += -DUSE_HAL_DRIVER -D$(CHIP) -D$(BOARD)
CFLAGS += -O3

LDFLAGS = -mcpu=cortex-m4 -mthumb
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,-static,--gc-sections,-Map=obj/output.map -nostartfiles
LIBS = -lm

ASFLAGS =

ARFLAGS = r

#
# By default reduce verbosity and don't print commands
# 'make V=1' will print the full commands
#

ifeq ("$(origin V)", "command line")
    ifeq ($(V),1)
        Q =
    else
        Q = @
    endif
else
    Q = @
endif

all: obj/x.bin

.PHONY: all

#
# Include dirs (absolute path)
#

INCDIRS = \
	-I $(SRC_ROOT)/../Inc/	\
	-I $(LIB_ROOT)/Drivers/STM32F4xx_HAL_Driver/Inc/	\
	-I $(LIB_ROOT)/Drivers/CMSIS/Device/ST/STM32F4xx/Include/	\
	-I $(LIB_ROOT)/Drivers/CMSIS/Include/	\
	-I $(LIB_ROOT)/Drivers/BSP/STM32F4xx-Nucleo/

#
# Source files (path relative to SRC_ROOT)
#

SRC_C = main.c stm32f4xx_hal_msp.c stm32f4xx_it.c system_stm32f4xx.c

#
# Library files (path relative to LIB_ROOT)
#

xLIB_C = \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c	\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c		\
	Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c
LIB_C = \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c		\
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c		\
	Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c


LIB_S = Projects/STM32F411RE-Nucleo/Templates/TrueSTUDIO/startup_stm32f411xe.s

STUB_C = newlib_stubs.c

#
# Compile rules
#

SRC_C_O = $(patsubst %.c,obj/%.o,$(SRC_C))
LIB_S_O = $(patsubst %.s,obj/%.o,$(LIB_S))
LIB_C_O = $(patsubst %.c,obj/%.o,$(LIB_C))
STUB_C_O = $(patsubst %.c,obj/%.o,$(STUB_C))

$(SRC_C_O): obj/%.o: $(SRC_ROOT)/%.c
	@echo Compile $<
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CFLAGS) $(INCDIRS) -MD -c -o $@ $<

$(LIB_S_O): obj/%.o: $(LIB_ROOT)/%.s
	@echo Compile $<
	$(Q)mkdir -p $(dir $@)
	$(Q)$(AS) $(ASFLAGS) -c -o $@ $<

$(LIB_C_O): obj/%.o: $(LIB_ROOT)/%.c
	@echo Compile $<
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CFLAGS) $(INCDIRS) -MD -c -o $@ $<

obj/x.elf: $(SRC_C_O) $(LIB_S_O) $(LIB_C_O) $(STUB_C_O)
	@echo Link $@
	$(Q)$(LD) $(LDFLAGS) -T $(LD_SCRIPT)	\
	 $(SRC_C_O) $(LIB_S_O) $(LIB_C_O) $(STUB_C_O)	\
	 $(LIBS) -o $@

$(STUB_C_O): obj/%.o: %.c
	@echo Compile $<
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CC) $(CFLAGS) $(INCDIRS) -MD -c -o $@ $<

obj/x.bin: obj/x.elf
	@echo Image $@
	$(Q)$(OBJCOPY) -O binary $< $@

clean:
	@echo Cleanup ...
	$(Q)rm -rf obj
