export ROOT_DIR := .
export BUILD_DIR := ./build
export BUILD_ROOT :=$(BUILD_DIR)


# common CFLAGS and LDFLAGS, BUILD_TYPE (debug, release)
include common-defs.mk

include app.mk

.PHONY: all firmware clean flash

all: config firmware

config:
	@echo "=== Build config =================="
	@echo " BUILD_TYPE = $(BUILD_TYPE)"
	@echo "=================================="

firmware:
	$(MAKE) -f firmware.mk

flash: firmware
	openocd -f interface/stlink-v2.cfg -f target/stm32f0x.cfg -c init -c targets -c 'reset halt' -c 'flash write_image erase ./build/fw-3_phase.hex' -c 'reset run' -c shutdown

clean::
	$(MAKE) -f firmware.mk $@
	
include build.mk
