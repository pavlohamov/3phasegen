# toolchain defs
include $(ROOT_DIR)/toolchain.mk

OUT_SFX := 3_phase
PROGNAME := fw-$(OUT_SFX)
BUILD_DIR :=$(BUILD_DIR)/$(OUT_SFX)

FIRMWARE := $(PROGNAME).hex
MAP_FILE := $(BUILD_ROOT)/$(PROGNAME).map

# firmware specific sources
export ASM_SRC := 

SRC += \
	$(wildcard ./firmware/*.c) \
	$(wildcard ./firmware/*.cpp) \
	
SRC += \
	./sdk/src/cmsis/system_stm32f0xx.c \
	./sdk/src/cmsis/vectors_stm32f0xx.c \
	./sdk/src/cortexm/_initialize_hardware.c \
	./sdk/src/cortexm/_reset_hardware.c \
	./sdk/src/cortexm/exception_handlers.c \
	\
	./sdk/src/diag/Trace.c \
	./sdk/src/diag/trace_impl.c \
	\
	$(wildcard ./sdk/src/newlib/*.c) \
	$(wildcard ./sdk/src/newlib/*.cpp) \

CFLAGS += \
	-I./firmware/ \
	-DTRACE \
	-DOS_USE_TRACE_SEMIHOSTING_STDOUT \

CFLAGS += \
	-Og \
	-ggdb \
	-mcpu=cortex-m0 \
	-mthumb \
	-ffunction-sections \
	-fdata-sections \
	-fmessage-length=0 \
	-fsigned-char \
	-ffreestanding \
	-fno-move-loop-invariants \

CXXFLAGS += \
	-fno-exceptions \
	-fno-rtti \
	-fno-use-cxa-atexit \
	-fno-threadsafe-statics \

LDFLAGS += \
	-Wl,--wrap=malloc \
	-Wl,--wrap=free \
	-mcpu=cortex-m0 \
	-mthumb \
	-T mem.ld -T libs.ld -T sections.ld \
	-nostartfiles \
	-Xlinker --gc-sections \
	-L"./ldscripts" \
	-Wl,-Map,"$(MAP_FILE)" \
	--specs=nano.specs \

.PHONY: all clean info

all: info $(FIRMWARE)

info:
	@echo "[BUILD] firmware"

$(FIRMWARE): $(PROGNAME)
	@echo " [FW] $< -> $@"
	$(OBJCOPY) -O ihex "$(BUILD_ROOT)/$(PROGNAME)" "$(BUILD_ROOT)/$(PROGNAME).hex"
	@echo " [Size]"
	$(SIZE) --format=berkeley "$(BUILD_ROOT)/$(PROGNAME)"

clean::
	@echo "[CLEAN] $(PROGNAME)"
	rm -rf $(BUILD_ROOT)/$(PROGNAME) $(BUILD_ROOT)/$(FIRMWARE) $(BUILD_ROOT)/$(MAP_FILE)

include $(ROOT_DIR)/build.mk
