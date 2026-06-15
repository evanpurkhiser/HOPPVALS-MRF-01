# Wrapper around idf.py so you don't have to source export.sh every time.
# IDF lives in ~/esp/esp-idf; toolchain in ~/.local/espressif.

IDF_PATH       ?= $(HOME)/esp/esp-idf
IDF_TOOLS_PATH ?= $(HOME)/.local/espressif
TARGET         ?= esp32c6
# XIAO ESP32-C6 enumerates as a usbmodem* device over native USB-Serial-JTAG.
# Override with `make flash PORT=/dev/cu.usbmodemXXXX` if auto-detect picks wrong.
PORT           ?= $(firstword $(wildcard /dev/cu.usbmodem*))

export IDF_TOOLS_PATH

# Run any command inside the IDF environment. `. export.sh` is required for
# PATH + IDF_PYTHON_ENV_PATH; we silence its banner for cleaner output.
IDF_RUN = . "$(IDF_PATH)/export.sh" >/dev/null && \
          IDF_TARGET=$(TARGET)

# Pass -p only if we found a port (otherwise idf.py errors out helpfully).
ifeq ($(PORT),)
  PORT_ARG :=
else
  PORT_ARG := -p $(PORT)
endif

.PHONY: help build flash monitor flash-monitor clean fullclean menuconfig erase set-target size reconfigure shell

help:
	@echo "Targets:"
	@echo "  build          - compile firmware"
	@echo "  flash          - flash firmware over USB-JTAG (PORT=$(PORT))"
	@echo "  monitor        - attach serial monitor (Ctrl-] to exit)"
	@echo "  flash-monitor  - flash then immediately attach monitor"
	@echo "  menuconfig     - interactive Kconfig UI"
	@echo "  erase          - full-chip erase (wipes NVS, Zigbee join state, etc.)"
	@echo "  clean          - remove build artifacts for current target"
	@echo "  fullclean      - nuke build/ and sdkconfig"
	@echo "  size           - flash + RAM usage summary"
	@echo "  set-target     - re-run idf.py set-target $(TARGET) (after big config changes)"
	@echo "  shell          - drop into a shell with IDF env loaded"

build:
	@$(IDF_RUN) idf.py build

flash:
	@$(IDF_RUN) idf.py $(PORT_ARG) flash

monitor:
	@$(IDF_RUN) idf.py $(PORT_ARG) monitor

flash-monitor:
	@$(IDF_RUN) idf.py $(PORT_ARG) flash monitor

menuconfig:
	@$(IDF_RUN) idf.py menuconfig

erase:
	@$(IDF_RUN) idf.py $(PORT_ARG) erase-flash

clean:
	@$(IDF_RUN) idf.py clean

fullclean:
	@$(IDF_RUN) idf.py fullclean

reconfigure:
	@$(IDF_RUN) idf.py reconfigure

set-target:
	@$(IDF_RUN) idf.py set-target $(TARGET)

size:
	@$(IDF_RUN) idf.py size

shell:
	@$(IDF_RUN) exec $$SHELL
