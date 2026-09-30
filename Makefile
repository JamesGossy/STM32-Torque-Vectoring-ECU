# Shortcuts for the ECU firmware. The real build is CMake in ECU_Firmware/.
#
#   make firmware       STM32 build (needs arm-none-eabi-gcc, CMake, Ninja)
#   make flash          program the board over SWD with STM32CubeProgrammer
#   make sim            host build: ecu_sim and the tests
#   make test           run the tests (add MC_DIR=... to include the motor controller interop test)
#   make format         clang-format every hand-written C file
#   make format-check   fail if anything is off-style, for CI
SHELL := /bin/sh
FW = ECU_Firmware
MC_DIR ?=
CLANG_FORMAT ?= clang-format

# The USB driver and syscalls are copied from the motor controller repo, so they keep its style.
FORMAT_SRCS = $(filter-out %/usb_cdc.c %/usb_desc.c %/usb_desc.h %/usb_cdc.h %/usb_ll.h %/syscalls.c, \
    $(wildcard $(FW)/app/*.[ch] $(FW)/hal/*.h $(FW)/hal/stm32g474/*.[ch] $(FW)/sim/*.[ch] \
    $(FW)/tests/*.[ch] $(FW)/tests/interop/*.c))

.PHONY: firmware flash sim test format format-check
firmware:
	cd $(FW) && cmake --preset default && cmake --build build
flash: firmware
	cd $(FW) && cmake --build build --target flash
sim:
	cd $(FW) && cmake --preset sim $(if $(MC_DIR),-DMC_FIRMWARE_DIR=$(abspath $(MC_DIR))) && cmake --build build-sim
test: sim
	cd $(FW) && ctest --preset sim
format:
	"$(CLANG_FORMAT)" -i $(FORMAT_SRCS)
format-check:
	"$(CLANG_FORMAT)" --dry-run --Werror $(FORMAT_SRCS)
