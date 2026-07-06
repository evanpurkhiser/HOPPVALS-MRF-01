# Wrapper around idf.py so you don't have to source export.sh every time.
# ESP-IDF and its toolchain live next to the repo, similar to a Python .venv.

REPO_ROOT      := $(abspath $(CURDIR)/..)
IDF_VERSION    ?= v5.3.2
IDF_REPO       ?= https://github.com/espressif/esp-idf.git
IDF_PATH       ?= $(REPO_ROOT)/.esp-idf
IDF_TOOLS_PATH ?= $(REPO_ROOT)/.idf_tools
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

# Over-the-air push: upload the built image to a device in WiFi debug mode.
# Find IP in the device's debug-mode boot log. Add `?normal=1` via NORMAL=1 to
# boot back into Zigbee instead of staying in debug mode for the next push.
OTA_BIN ?= build/hv-mrf-01.bin
OTA_URL  = http://$(IP)/ota$(if $(NORMAL),?normal=1,)

.PHONY: help bootstrap build flash monitor flash-monitor clean fullclean menuconfig erase set-target size reconfigure shell ota

help:
	@echo "Targets:"
	@echo "  bootstrap      - install ESP-IDF $(IDF_VERSION) + tools into repo-local paths"
	@echo "  build          - compile firmware"
	@echo "  flash          - flash firmware over USB-JTAG (PORT=$(PORT))"
	@echo "  monitor        - attach serial monitor (Ctrl-] to exit)"
	@echo "  flash-monitor  - flash then immediately attach monitor"
	@echo "  ota IP=<ip>    - push firmware over WiFi to a device in debug mode (NORMAL=1 to exit debug)"
	@echo "  menuconfig     - interactive Kconfig UI"
	@echo "  erase          - full-chip erase (wipes NVS, Zigbee join state, etc.)"
	@echo "  clean          - remove build artifacts for current target"
	@echo "  fullclean      - nuke build/ and sdkconfig"
	@echo "  size           - flash + RAM usage summary"
	@echo "  set-target     - re-run idf.py set-target $(TARGET) (after big config changes)"
	@echo "  shell          - drop into a shell with IDF env loaded"

bootstrap:
	@test -d "$(IDF_PATH)" || git clone --recursive --depth 1 --shallow-submodules -b "$(IDF_VERSION)" "$(IDF_REPO)" "$(IDF_PATH)"
	@IDF_TOOLS_PATH="$(IDF_TOOLS_PATH)" "$(IDF_PATH)/install.sh" "$(TARGET)"

build:
	@$(IDF_RUN) idf.py build

flash:
	@$(IDF_RUN) idf.py $(PORT_ARG) flash

monitor:
	@$(IDF_RUN) idf.py $(PORT_ARG) monitor

flash-monitor:
	@$(IDF_RUN) idf.py $(PORT_ARG) flash monitor

# ── Over-the-air firmware update (no USB) ───────────────────────────────────
#
# Flow:
#   1. Put the device in WiFi debug mode: flip the "Debug Mode" switch in Home
#      Assistant (ZHA), or run `debug` on the console (USB or websocket). It
#      reboots, joins WiFi (creds from NVS), and logs its IP + ws://<ip>/ws.
#      (WiFi creds are set once over USB: `config set net.ssid ...` / `net.pass`,
#      then `config save`.)
#   2. Build:  make build
#   3. Push:   make ota IP=<device-ip>
#      -> POSTs build/hv-mrf-01.bin to http://<ip>/ota.
#
# What the device does on a push (see components/http_debug):
#   - De-energizes the motors for the whole write + reboot (they can't run).
#   - Writes the *inactive* OTA slot (ota_0/ota_1), then reboots into it.
#   - Re-arms debug mode by DEFAULT, so it comes back on WiFi ready for the next
#     push. Pass NORMAL=1 (-> ?normal=1) to boot into normal Zigbee mode instead,
#     e.g. when you're done iterating:  make ota IP=<ip> NORMAL=1
#
# Safety nets:
#   - Bootloader rollback: an image that can't boot or get on WiFi auto-reverts
#     to the previous slot (it's marked valid only once it proves healthy — the
#     debug server coming up, or a Zigbee rejoin in normal mode).
#   - Recovery fallback: a normal boot that never joins Zigbee within ~5 min
#     reboots itself into WiFi debug mode, so a Zigbee-side fault can't strand a
#     device with no USB access.
#
# Note: applying the ota_0/ota_1 partition layout itself required one USB flash;
# after that, updates go over WiFi. A full `make flash` (not app-only) is needed
# if the partition table ever changes again.
ota:
	@test -n "$(IP)" || { echo "usage: make ota IP=<device-ip> [NORMAL=1]"; exit 1; }
	@echo "Pushing $(OTA_BIN) → $(OTA_URL)"
	curl -fsS --data-binary @"$(OTA_BIN)" "$(OTA_URL)"

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
