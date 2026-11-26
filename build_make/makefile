################################################################################
# Course template entry point: `make -j8` here writes
# build_make/mtk3pico_smp1_usb_cdc_wifi.uf2.
#
# The firmware image is described once, in ../CMakeLists.txt. This makefile
# configures that build on first use and then builds its `main` target, so the
# two builds cannot drift apart. Run it inside `nix develop` (or with the Arm
# toolchain, CMake and PICO_SDK_PATH set as in ../README.md).
################################################################################

IMAGE     := mtk3pico_smp1_usb_cdc_wifi
BUILD_DIR := ../build

# Template options. This car has one image: SMP on both cores, the USB console
# and the CYW43 radio. TEST self-tests do not exist here; host tests run with
# CTest. CRASH_REPORT=1 would take GP0/GP1, which the sonar uses.
SMP          ?= 1
CONSOLE      ?= usb_cdc
WIFI         ?= cyw43
TEST         ?= 0
CRASH_REPORT ?= 0

ifneq ($(SMP) $(CONSOLE) $(WIFI),1 usb_cdc cyw43)
$(error This firmware builds only SMP=1 CONSOLE=usb_cdc WIFI=cyw43)
endif
ifneq ($(TEST),0)
$(error TEST=1 is not supported; run the host tests with CTest)
endif
ifneq ($(CRASH_REPORT),0)
$(error CRASH_REPORT=1 is not supported: the sonar uses GP0 and GP1)
endif

.DEFAULT_GOAL := all
.PHONY: all clean

all: $(BUILD_DIR)/CMakeCache.txt
	+cmake --build $(BUILD_DIR) --target main
	cp $(BUILD_DIR)/main.uf2 $(IMAGE).uf2
	@echo 'Finished building: $(IMAGE).uf2'

$(BUILD_DIR)/CMakeCache.txt:
	cd .. && ./configure.sh

clean:
	rm -f $(IMAGE).uf2
