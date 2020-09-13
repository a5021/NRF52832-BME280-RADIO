PROJECT_NAME     := NRF52832-BME280-RADIO
TARGETS          := NRF52832-BME280-RADIO
OUTPUT_DIRECTORY := _build

PROJ_DIR := .

$(OUTPUT_DIRECTORY)/NRF52832-BME280-RADIO.out: \
  LINKER_SCRIPT := $(PROJ_DIR)/IDE/GCC/ld/gcc_nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(PROJ_DIR)/drv/src/gcc_startup_nrf52.S \
  $(PROJ_DIR)/drv/src/system_nrf52.c \
  $(PROJ_DIR)/main.c

# Include folders common to all targets
INC_FOLDERS += \
  $(PROJ_DIR)/drv/inc/CMSIS \
  $(PROJ_DIR)/drv/inc \

# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -Ofast -g0
# Uncomment the line below to enable link time optimization
OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT) -fverbose-asm
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Wextra -Werror -Wpedantic -std=gnu11
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
#CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -ffunction-sections -fdata-sections -fstrict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g0 -fverbose-asm
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Linker flags
LDFLAGS += $(OPT) -fverbose-asm
LDFLAGS += -mthumb -mabi=aapcs -L$(PROJ_DIR)/IDE/GCC/ld -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

NRF52832-BME280-RADIO: CFLAGS += -D__HEAP_SIZE=2048
NRF52832-BME280-RADIO: CFLAGS += -D__STACK_SIZE=2048
NRF52832-BME280-RADIO: ASMFLAGS += -D__HEAP_SIZE=2048
NRF52832-BME280-RADIO: ASMFLAGS += -D__STACK_SIZE=2048

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm

STLINK = ST-LINK_CLI.exe
#STLINK_FLAGS = -c UR -V -P $< -Hardrst -Run
STLINK_FLAGS = -c SWD -V -ME -P $< -Hardrst -Run

.PHONY: default help

# Default target - first one defined
default: NRF52832-BME280-RADIO

TEMPLATE_PATH := $(PROJ_DIR)/IDE/GCC/mk

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash erase

# Flash the program
jflash: $(OUTPUT_DIRECTORY)/NRF52832-BME280-RADIO.hex
	@echo Flashing: $<
	nrfjprog -f nrf52 --program $< --sectorerase --verify --log 
	nrfjprog -f nrf52 --reset

stflash: $(OUTPUT_DIRECTORY)/NRF52832-BME280-RADIO.hex
	@echo Flashing: $<
	$(STLINK) $(STLINK_FLAGS)

erase:
	nrfjprog -f nrf52 --eraseall
