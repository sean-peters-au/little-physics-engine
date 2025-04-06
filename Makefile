# @fileoverview Makefile
# @brief Build script for the physics simulator.

# ---------------------------------------------------------------------------------
# Common settings
# ---------------------------------------------------------------------------------

# Only apply macOS-specific flags
ifeq ($(shell uname),Darwin)
APP_CXXFLAGS += -stdlib=libc++ -isysroot $(shell xcrun --show-sdk-path) -D_DARWIN_C_SOURCE -D_XOPEN_SOURCE=700
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
ASSETS_DIR := assets

# Source files
SRCS := $(shell find $(SRC_DIR) -name '*.cpp')

# Object files (handle Objective-C++ separately)
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(filter-out $(SRC_DIR)/renderers/fluid_renderer.cpp,$(SRCS))) \
        $(BUILD_DIR)/renderers/fluid_renderer.o

# Dependency files
DEPS := $(OBJS:.o=.d)

# Clang-tidy settings
TIDY := run-clang-tidy
TIDY_FIX_FLAGS := -fix

# ---------------------------------------------------------------------------------
# Build Target
# ---------------------------------------------------------------------------------

.DEFAULT_GOAL := simulator

.PHONY: simulator print-debug clean rebuild directories copy_assets lint lint-fix compile_commands.json

print-debug:
	@echo "SRCS: $(SRCS)"
	@echo "OBJS: $(OBJS)"
	@echo "DEPS: $(DEPS)"

# Compile fluid_renderer.cpp as Objective-C++
$(BUILD_DIR)/renderers/fluid_renderer.o: $(SRC_DIR)/renderers/fluid_renderer.cpp | directories
	@echo "Compiling Objective-C++ $<"
	$(CXX) $(CXXFLAGS) -x objective-c++ -MMD -MP -MF $(@:.o=.d) -c $< -o $@

# General rule for C++ files (excluding the Objective-C++ one)
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | directories
	@echo "Compiling C++ $<"
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(@:.o=.d) -c $< -o $@

simulator: CXX := clang++
simulator: CXXFLAGS += $(APP_CXXFLAGS) -Xpreprocessor -fopenmp -I$(LIBOMP_PREFIX)/include $(METAL_INCLUDE) $(METAL_CPP_INCLUDE)
simulator: LDFLAGS := -L/opt/homebrew/lib -L$(LIBOMP_PREFIX)/lib -lsfml-graphics -lsfml-window -lsfml-system -lomp -framework Metal -framework Cocoa -framework MetalKit -framework QuartzCore
simulator: $(OBJS) $(BUILD_DIR)/fluid_kernels.metallib $(BUILD_DIR)/fluid_renderer_kernels.metallib
	@echo "Building simulator target with objects: $(OBJS)"
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(BUILD_DIR)/simulator $(LDFLAGS)

# Include generated dependency files if they exist
-include $(DEPS)

# ---------------------------------------------------------------------------------
# Utility targets
# ---------------------------------------------------------------------------------

# Create all necessary build subdirectories based on src structure
BUILD_DIRS := $(BUILD_DIR) $(patsubst $(SRC_DIR)/%,$(BUILD_DIR)/%,$(shell find $(SRC_DIR) -mindepth 1 -type d))
directories:
	@echo "Creating build directories..."
	@mkdir -p $(BUILD_DIRS)
	@mkdir -p $(ASSETS_DIR)/fonts

copy_assets: directories
	@if [ ! -f $(ASSETS_DIR)/fonts/arial.ttf ]; then \
		echo "Downloading Arial font..."; \
		curl -L "https://github.com/matomo-org/travis-scripts/raw/master/fonts/Arial.ttf" \
			-o $(ASSETS_DIR)/fonts/arial.ttf; \
	fi

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean simulator

compile_commands.json: directories
	@echo "Generating compile_commands.json..."
	@$(MAKE) simulator # Ensure target is built first
	@cd $(BUILD_DIR) && \
	CXXFLAGS="$(CXXFLAGS)" bear -- $(MAKE) -C .. -B simulator > /dev/null 2>&1 || true
	@echo "Generated compile_commands.json"

# ---------------------------------------------------------------------------------
# Clang-Tidy Targets
# ---------------------------------------------------------------------------------
# We explicitly list only our own source files (SRCS),
# and we override header-filter to skip /opt/homebrew and vendor.
# On macOS, use 'sysctl -n hw.ncpu' for parallel jobs; on Linux, 'nproc' is fine.

lint: compile_commands.json
	@echo "Running clang-tidy..."
	@echo "Checking build..."
	$(TIDY) -header-filter='^((?!vendor|/opt/homebrew).)*$$' \
		-p $(BUILD_DIR) -quiet -j $(shell sysctl -n hw.ncpu) \
		$(SRCS); \

lint-fix: compile_commands.json
	@echo "Running clang-tidy with auto-fix..."
	@echo "Checking build..."
	$(TIDY) -header-filter='^((?!vendor|/opt/homebrew).)*$$' \
		-p $(BUILD_DIR) $(TIDY_FIX_FLAGS) \
		$(SRCS); \

# ---------------------------------------------------------------------------------
# Build rule for Metal shader files
# ---------------------------------------------------------------------------------

# Rule for PHYSICS kernels
$(BUILD_DIR)/fluid_kernels.metallib: $(SRC_DIR)/systems/fluid/fluid_kernels.metal | directories
	@echo "Compiling PHYSICS Metal shader $<"
	$(METAL) -c $< -I./include -o $(BUILD_DIR)/fluid_kernels.air
	$(METALLIB) $(BUILD_DIR)/fluid_kernels.air -o $@
	@rm $(BUILD_DIR)/fluid_kernels.air

# Rule for RENDERING kernels
$(BUILD_DIR)/fluid_renderer_kernels.metallib: $(SRC_DIR)/renderers/fluid_renderer_kernels.metal | directories
	@echo "Compiling RENDERING Metal shader $<"
	$(METAL) -c $< -I./include -o $(BUILD_DIR)/fluid_renderer_kernels.air
	$(METALLIB) $(BUILD_DIR)/fluid_renderer_kernels.air -o $@
	@rm $(BUILD_DIR)/fluid_renderer_kernels.air