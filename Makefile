# Compiler settings
CXX := clang++
CXXFLAGS := -Wall -Wextra -std=c++17 \
            -I./include \
            -I./include/nbody/vendor/entt/include \
            -ffast-math -mrecip -march=native \
            -I/opt/homebrew/include
LDFLAGS := -L/opt/homebrew/lib -lsfml-graphics -lsfml-window -lsfml-system

# Directory structure
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
TEST_DIR := tests
ASSETS_DIR := assets

# Find all source files recursively (excluding main.cpp for tests)
SRCS := $(shell find $(SRC_DIR) -name '*.cpp' ! -name 'main.cpp')
OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)

# Test sources
TEST_SRCS := $(shell find $(TEST_DIR) -name '*.cpp')
TEST_OBJS := $(TEST_SRCS:$(TEST_DIR)/%.cpp=$(BUILD_DIR)/tests/%.o)

# Main target
TARGET := $(BUILD_DIR)/simulator
TEST_TARGET := $(BUILD_DIR)/test_runner

# Default target
all: directories copy_assets $(TARGET)

# Create necessary directories
directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/systems
	@mkdir -p $(BUILD_DIR)/core
	@mkdir -p $(BUILD_DIR)/rendering
	@mkdir -p $(BUILD_DIR)/tests
	@mkdir -p $(ASSETS_DIR)/fonts

# Download and copy assets
copy_assets: directories
	@if [ ! -f $(ASSETS_DIR)/fonts/arial.ttf ]; then \
		echo "Downloading Arial font..."; \
		curl -L "https://github.com/matomo-org/travis-scripts/raw/master/fonts/Arial.ttf" \
			-o $(ASSETS_DIR)/fonts/arial.ttf; \
	fi

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile test files
$(BUILD_DIR)/tests/%.o: $(TEST_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Link the main executable
$(TARGET): $(OBJS) $(BUILD_DIR)/main.o
	$(CXX) $^ -o $(TARGET) $(LDFLAGS)

# Link the test executable
$(TEST_TARGET): $(OBJS) $(TEST_OBJS)
	$(CXX) $^ -o $(TEST_TARGET) $(LDFLAGS) -lgtest -lgtest_main -pthread

# Build and run tests
test: $(TEST_TARGET)
	./$(TEST_TARGET)

# Clean build files
clean:
	rm -rf $(BUILD_DIR)

# Clean and rebuild
rebuild: clean all

.PHONY: all clean rebuild test directories copy_assets
