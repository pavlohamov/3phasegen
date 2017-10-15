export ROOT_DIR := .
export BUILD_DIR := ./build
export BUILD_ROOT :=$(BUILD_DIR)


# common CFLAGS and LDFLAGS, BUILD_TYPE (debug, release)
include common-defs.mk

include app.mk

.PHONY: all firmware clean

all: config firmware

config:
	@echo "=== Build config =================="
	@echo " BUILD_TYPE = $(BUILD_TYPE)"
	@echo "=================================="

firmware:
	$(MAKE) -f firmware.mk


clean::
	$(MAKE) -f firmware.mk $@
	
include build.mk
