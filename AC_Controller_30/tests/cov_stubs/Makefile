# =============================================================================
# Makefile — AC_Controller_30 C/C++ coverage build
#
# Run from the tests/cov_stubs/ directory on Linux/WSL:
#
#   make          — build and run the harness, generate HTML report
#   make build    — compile only
#   make run      — run the harness
#   make report   — generate lcov HTML report
#   make clean    — remove all build artifacts
# =============================================================================

# ── Paths (relative to this Makefile, which lives in tests/cov_stubs/) ───────
PROJ_ROOT  := ../..
MAIN_DIR   := $(PROJ_ROOT)/main
STUBS_DIR  := $(CURDIR)/stubs
BUILD_DIR  := $(CURDIR)/build
REPORT_DIR := $(CURDIR)/html_report

# ── Toolchain ─────────────────────────────────────────────────────────────────
CC  := gcc
CXX := g++

# ── Common flags ──────────────────────────────────────────────────────────────
CFLAGS_COMMON := \
    --coverage \
    -fprofile-arcs \
    -ftest-coverage \
    -fprofile-update=atomic \
    -g \
    -O0 \
    -Wall \
    -Wno-unused-variable \
    -Wno-unused-function \
    -Wno-missing-braces \
    -Wno-implicit-function-declaration \
    -Wno-int-conversion \
    -Wno-incompatible-pointer-types \
    -Wno-discarded-qualifiers \
    -Wno-pointer-sign \
    -Wno-format \
    -include netinet/in.h \
    -include sys/stat.h \
    -DCONFIG_IDF_TARGET_ESP32S3=1 \
    -DESP_PLATFORM=1 \
    -DLOG_LOCAL_LEVEL=3 \
    -D_POSIX_THREADS \
    -I$(STUBS_DIR) \
    -I$(MAIN_DIR) \
    -I$(MAIN_DIR)/SafeFunctions \
    -I$(MAIN_DIR)/modbus \
    -I$(MAIN_DIR)/wifi \
    -I$(MAIN_DIR)/mqtt \
    -I$(MAIN_DIR)/controller \
    -I$(MAIN_DIR)/TimeManagement \
    -I$(MAIN_DIR)/uart \
    -I$(MAIN_DIR)/ota \
    -I$(MAIN_DIR)/xmodem \
    -I$(MAIN_DIR)/sensors \
    -I$(MAIN_DIR)/ble_spp \
    -I$(MAIN_DIR)/configuration

CFLAGS   := $(CFLAGS_COMMON) -std=gnu11
CXXFLAGS := $(CFLAGS_COMMON) -std=gnu++17 -Wno-write-strings

LDFLAGS  := --coverage -lm

# ── Source files compiled from their ORIGINAL locations ──────────────────────
#    Each object is placed in build/ to keep coverage .gcno/.gcda there.

C_SOURCES := \
    $(MAIN_DIR)/modbus/data.c \
    $(MAIN_DIR)/modbus/modbus_rtu.c \
    $(MAIN_DIR)/modbus/modbus_tcp.c \
    $(MAIN_DIR)/modbus/nanomodbus.c \
    $(MAIN_DIR)/TimeManagement/TimeManagement.c \
    $(MAIN_DIR)/configuration/configuration.c \
    $(MAIN_DIR)/wifi/wifiConnect.c \
    $(MAIN_DIR)/mqtt/mqtt_manager.c \
    $(MAIN_DIR)/controller/controller.c \
    $(MAIN_DIR)/ota/ota.c \
    $(MAIN_DIR)/uart/uart_app.c \
    $(MAIN_DIR)/xmodem/xmodem.c \
    $(MAIN_DIR)/sensors/sensors.c \
    $(MAIN_DIR)/sensors/sensirion_gas_index_algorithm.c \
    $(MAIN_DIR)/ble_spp/ble_spp_server_demo.c \
    $(MAIN_DIR)/main.c

CXX_SOURCES := \
    $(MAIN_DIR)/SafeFunctions/Jio_EA_Safe_Functions.cpp

HARNESS_SOURCES := \
    $(CURDIR)/test_main.c \
    $(CURDIR)/esp_stubs.c \
    $(CURDIR)/global_vars.c \
    $(CURDIR)/cJSON.c

# ── Derived object lists ───────────────────────────────────────────────────────
C_OBJECTS   := $(patsubst $(PROJ_ROOT)/%.c,   $(BUILD_DIR)/%.o, $(C_SOURCES))
CXX_OBJECTS := $(patsubst $(PROJ_ROOT)/%.cpp, $(BUILD_DIR)/%.o, $(CXX_SOURCES))
H_OBJECTS   := $(patsubst $(CURDIR)/%.c,      $(BUILD_DIR)/harness/%.o, $(HARNESS_SOURCES))

ALL_OBJECTS := $(C_OBJECTS) $(CXX_OBJECTS) $(H_OBJECTS)

TARGET := $(BUILD_DIR)/test_coverage

# ── Default target ─────────────────────────────────────────────────────────────
.PHONY: all build run report clean

all: build run report

build: $(TARGET)

# ── Link ───────────────────────────────────────────────────────────────────────
$(TARGET): $(ALL_OBJECTS)
	@echo "[LINK] $@"
	$(CXX) $(ALL_OBJECTS) $(LDFLAGS) -o $@

# ── Compile C source files from main/ ──────────────────────────────────────────
$(BUILD_DIR)/%.o: $(PROJ_ROOT)/%.c
	@mkdir -p $(dir $@)
	@echo "[CC] $<"
	$(CC) $(CFLAGS) -c $< -o $@

# ── Compile C++ source files from main/ ────────────────────────────────────────
$(BUILD_DIR)/%.o: $(PROJ_ROOT)/%.cpp
	@mkdir -p $(dir $@)
	@echo "[CXX] $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ── Compile harness files ──────────────────────────────────────────────────────
$(BUILD_DIR)/harness/%.o: $(CURDIR)/%.c
	@mkdir -p $(dir $@)
	@echo "[CC-harness] $<"
	$(CC) $(CFLAGS) -c $< -o $@

# ── Run ───────────────────────────────────────────────────────────────────────
run: build
	@echo "[RUN] $(TARGET)"
	@$(TARGET) || true

# ── Coverage report ───────────────────────────────────────────────────────────
report: run
	@echo "[LCOV] Capturing coverage data..."
	@mkdir -p $(REPORT_DIR)
	lcov \
	    --capture \
	    --directory $(BUILD_DIR) \
	    --output-file $(BUILD_DIR)/coverage.info \
	    --rc branch_coverage=1 \
	    --ignore-errors mismatch,unused \
	    2>/dev/null || \
	lcov \
	    --capture \
	    --directory $(BUILD_DIR) \
	    --output-file $(BUILD_DIR)/coverage.info \
	    2>/dev/null
	@echo "[LCOV] Filtering to main/ sources only..."
	lcov \
	    --remove $(BUILD_DIR)/coverage.info \
	    '*/stubs/*' \
	    '*/usr/*' \
	    '/usr/*' \
	    '*/cov_stubs/*' \
	    --output-file $(BUILD_DIR)/coverage_filtered.info \
	    2>/dev/null || \
	    cp $(BUILD_DIR)/coverage.info $(BUILD_DIR)/coverage_filtered.info
	@echo "[GENHTML] Generating HTML report in $(REPORT_DIR) ..."
	genhtml \
	    $(BUILD_DIR)/coverage_filtered.info \
	    --output-directory $(REPORT_DIR) \
	    --title "AC_Controller_30 Coverage" \
	    --legend \
	    --branch-coverage \
	    2>/dev/null || \
	genhtml \
	    $(BUILD_DIR)/coverage_filtered.info \
	    --output-directory $(REPORT_DIR) \
	    --title "AC_Controller_30 Coverage"
	@echo ""
	@echo "=========================================="
	@echo " HTML report: $(REPORT_DIR)/index.html"
	@echo "=========================================="

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -rf $(BUILD_DIR) $(REPORT_DIR)
