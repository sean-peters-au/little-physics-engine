# @fileoverview Makefile
# @brief Build script for both native (desktop) and WASM (web) targets.

# ---------------------------------------------------------------------------------
# Common settings
# ---------------------------------------------------------------------------------

# Only apply macOS-specific flags for native builds
ifeq ($(shell uname),Darwin)
NATIVE_CXXFLAGS += -stdlib=libc++ -isysroot $(shell xcrun --show-sdk-path) -D_DARWIN_C_SOURCE -D_XOPEN_SOURCE=700
endif

# Determine the Homebrew prefix for libomp
LIBOMP_PREFIX := $(shell brew --prefix libomp)

# Include entt
ENTT_INCLUDE := -I./vendor/entt/include

# Base flags for all builds
CXXFLAGS := -Wall -Wextra -std=c++17 \
            -I./include \
            -isystem /opt/homebrew/include \
			$(ENTT_INCLUDE)

# Define path for Metal framework headers on macOS.
METAL_INCLUDE := -I$(shell xcrun --sdk macosx --show-sdk-path)/System/Library/Frameworks/Metal.framework/Headers

# Add metal-cpp include directory (make sure you have downloaded metal-cpp!)
METAL_CPP_INCLUDE := -I./vendor/metal-cpp

# Define Metal compiler variables (for compiling .metal shaders)
METAL := xcrun -sdk macosx metal
METALLIB := xcrun -sdk macosx metallib

# Directory structure
BUILD_DIR := build
SRC_DIR := src
INC_DIR := include
TEST_DIR := tests
ASSETS_DIR := assets

# Base source files (excluding arch-specific code)
BASE_SRCS := $(shell find $(SRC_DIR) -name '*.cpp' \
             ! -path "$(SRC_DIR)/arch/*")

# Clang-tidy settings
TIDY := run-clang-tidy
TIDY_FIX_FLAGS := -fix

# ---------------------------------------------------------------------------------
# Architecture-specific targets
# ---------------------------------------------------------------------------------

.PHONY: native wasm print-debug

print-debug:
	@echo "BASE_SRCS: $(BASE_SRCS)"
	@echo "ARCH_SRCS: $(ARCH_SRCS)"
	@echo "OBJS: $(OBJS)"

# Define variables for native build
NATIVE_ARCH_SRCS := $(wildcard $(SRC_DIR)/arch/native/*.cpp)
NATIVE_OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/native/%.o,$(BASE_SRCS)) \
               $(patsubst $(SRC_DIR)/arch/native/%.cpp,$(BUILD_DIR)/arch/native/%.o,$(NATIVE_ARCH_SRCS))
NATIVE_SRCS := $(BASE_SRCS) $(NATIVE_ARCH_SRCS)

# Define variables for wasm build
WASM_ARCH_SRCS := $(wildcard $(SRC_DIR)/arch/wasm/*.cpp)
WASM_OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/wasm/%.o,$(BASE_SRCS)) \
             $(patsubst $(SRC_DIR)/arch/wasm/%.cpp,$(BUILD_DIR)/arch/wasm/%.o,$(WASM_ARCH_SRCS))
WASM_SRCS := $(BASE_SRCS) $(WASM_ARCH_SRCS)

native: CXX := clang++
native: CXXFLAGS += $(NATIVE_CXXFLAGS) -Xpreprocessor -fopenmp -I$(LIBOMP_PREFIX)/include $(METAL_INCLUDE) $(METAL_CPP_INCLUDE)
native: LDFLAGS := -L/opt/homebrew/lib -L$(LIBOMP_PREFIX)/lib -lsfml-graphics -lsfml-window -lsfml-system -lomp -framework Metal -framework Cocoa -framework MetalKit
native: directories copy_assets $(NATIVE_OBJS) $(BUILD_DIR)/fluid_kernels.metallib
	@echo "Building native target with objects: $(NATIVE_OBJS)"
	$(CXX) $(CXXFLAGS) $(NATIVE_OBJS) -o $(BUILD_DIR)/simulator_native $(LDFLAGS)

wasm: CXX := em++
wasm: CXXFLAGS += -s USE_WEBGL2=1  # Add emscripten-specific flags
wasm: LDFLAGS := -s USE_WEBGL2=1 -s FULL_ES3=1 \
                 -s ALLOW_MEMORY_GROWTH=1 \
                 -s INVOKE_RUN=1 -s DEMANGLE_SUPPORT=1
wasm: directories copy_assets $(WASM_OBJS)
	@echo "Building wasm target with objects: $(WASM_OBJS)"
	$(CXX) $(CXXFLAGS) $(WASM_OBJS) -o $(BUILD_DIR)/simulator_wasm.html \
	    $(LDFLAGS) \
	    --shell-file assets/sim.html \
	    -s EXIT_RUNTIME=1 -s ASSERTIONS=1

# ---------------------------------------------------------------------------------
# Build rules
# ---------------------------------------------------------------------------------

$(BUILD_DIR)/native/%.o: $(SRC_DIR)/%.cpp
	@echo "Compiling native $<"
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/wasm/%.o: $(SRC_DIR)/%.cpp
	@echo "Compiling wasm $<"
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/arch/native/%.o: $(SRC_DIR)/arch/native/%.cpp
	@echo "Compiling arch-specific native $<"
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/arch/wasm/%.o: $(SRC_DIR)/arch/wasm/%.cpp
	@echo "Compiling arch-specific wasm $<"
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ---------------------------------------------------------------------------------
# Test targets
# ---------------------------------------------------------------------------------

TEST_SRCS := $(shell find $(TEST_DIR) -name '*.cpp')
TEST_OBJS := $(TEST_SRCS:$(TEST_DIR)/%.cpp=$(BUILD_DIR)/tests/%.o)
TEST_TARGET := $(BUILD_DIR)/test_runner

test: $(TEST_TARGET)
	./$(TEST_TARGET)

$(TEST_TARGET): $(OBJS) $(TEST_OBJS)
	$(CXX) $^ -o $@ $(LDFLAGS) -lgtest -lgtest_main -pthread

$(BUILD_DIR)/tests/%.o: $(TEST_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ---------------------------------------------------------------------------------
# Utility targets
# ---------------------------------------------------------------------------------

directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/systems
	@mkdir -p $(BUILD_DIR)/core
	@mkdir -p $(BUILD_DIR)/rendering
	@mkdir -p $(BUILD_DIR)/tests
	@mkdir -p $(BUILD_DIR)/arch/native
	@mkdir -p $(BUILD_DIR)/arch/wasm
	@mkdir -p $(ASSETS_DIR)/fonts

copy_assets: directories
	@if [ ! -f $(ASSETS_DIR)/fonts/arial.ttf ]; then \
		echo "Downloading Arial font..."; \
		curl -L "https://github.com/matomo-org/travis-scripts/raw/master/fonts/Arial.ttf" \
			-o $(ASSETS_DIR)/fonts/arial.ttf; \
	fi

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean all

compile_commands.json: directories
	@echo "Generating compile_commands.json..."
	@cd $(BUILD_DIR) && \
	CXXFLAGS="$(CXXFLAGS)" bear -- $(MAKE) -C .. -B native > /dev/null 2>&1 || true
	@echo "Generated compile_commands.json"

# ---------------------------------------------------------------------------------
# Clang-Tidy Targets
# ---------------------------------------------------------------------------------
# We explicitly list only our own source files (NATIVE_SRCS or WASM_SRCS),
# and we override header-filter to skip /opt/homebrew and vendor.
# On macOS, use 'sysctl -n hw.ncpu' for parallel jobs; on Linux, 'nproc' is fine.

lint: compile_commands.json
	@echo "Running clang-tidy..."
	@if [ "$(WASM)" = "1" ]; then \
		echo "Checking WASM build..."; \
		$(TIDY) -header-filter='^((?!vendor|/opt/homebrew).)*$$' \
			-p $(BUILD_DIR) -quiet -j $(shell sysctl -n hw.ncpu) \
			$(WASM_SRCS); \
	else \
		echo "Checking native build..."; \
		$(TIDY) -header-filter='^((?!vendor|/opt/homebrew).)*$$' \
			-p $(BUILD_DIR) -quiet -j $(shell sysctl -n hw.ncpu) \
			$(NATIVE_SRCS); \
	fi

lint-fix: compile_commands.json
	@echo "Running clang-tidy with auto-fix..."
	@if [ "$(WASM)" = "1" ]; then \
		echo "Checking WASM build..."; \
		$(TIDY) -header-filter='^((?!vendor|/opt/homebrew).)*$$' \
			-p $(BUILD_DIR) $(TIDY_FIX_FLAGS) \
			$(WASM_SRCS); \
	else \
		echo "Checking native build..."; \
		$(TIDY) -header-filter='^((?!vendor|/opt/homebrew).)*$$' \
			-p $(BUILD_DIR) $(TIDY_FIX_FLAGS) \
			$(NATIVE_SRCS); \
	fi

# Add new serve target
serve: wasm
	@echo "Starting server at http://localhost:8080/simulator_wasm.html"
	@if command -v python3 &> /dev/null; then \
		if command -v xdg-open &> /dev/null; then \
			xdg-open http://localhost:8080/simulator_wasm.html; \
		elif command -v open &> /dev/null; then \
			open http://localhost:8080/simulator_wasm.html; \
		fi; \
		cd $(BUILD_DIR) && python3 -m http.server 8080; \
	else \
		echo "Error: python3 is not installed"; \
		exit 1; \
	fi

# ---------------------------------------------------------------------------------
# Build rule for Metal shader file
# ---------------------------------------------------------------------------------

$(BUILD_DIR)/fluid_kernels.metallib: $(SRC_DIR)/systems/fluid/fluid_kernels.metal | directories
	@echo "Compiling Metal shader $<"
	$(METAL) -c $< -o $(BUILD_DIR)/fluid_kernels.air
	$(METALLIB) $(BUILD_DIR)/fluid_kernels.air -o $@
	@rm $(BUILD_DIR)/fluid_kernels.air

.PHONY: all native wasm clean rebuild test directories copy_assets lint lint-fix compile_commands serve