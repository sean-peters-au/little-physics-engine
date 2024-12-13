# Compiler settings
CXX := clang++
CXXFLAGS := -Wall -Wextra -std=c++17 -I./include -I./external/entt/include -ffast-math -mrecip -march=native \
            -I/opt/homebrew/include
LDFLAGS := -L/opt/homebrew/lib -lsfml-graphics -lsfml-window -lsfml-system

# Directory structure
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
TEST_DIR := test
EXTERNAL_DIR := external
ASSETS_DIR := assets

# Find all source files
SRCS := $(wildcard $(SRC_DIR)/*.cpp) $(wildcard $(SRC_DIR)/systems/*.cpp)
OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)

# Main target
TARGET := $(BUILD_DIR)/simulator

# Default target
all: directories copy_assets $(TARGET)

# Create necessary directories
directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/systems
	@mkdir -p $(ASSETS_DIR)

# Download and copy assets
copy_assets: directories
	@if [ ! -f arial.ttf ]; then \
		echo "Downloading Arial font..."; \
		curl -L "https://github.com/matomo-org/travis-scripts/raw/master/fonts/Arial.ttf" -o $(ASSETS_DIR)/arial.ttf; \
		cp $(ASSETS_DIR)/arial.ttf ./arial.ttf; \
	fi

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(LDFLAGS)

# Test target
test: directories
	$(CXX) $(CXXFLAGS) $(TEST_DIR)/*.cpp -o $(BUILD_DIR)/test
	./$(BUILD_DIR)/test

# Clean build files
clean:
	rm -rf $(BUILD_DIR)
	rm -f arial.ttf

# Clean and rebuild
rebuild: clean all

.PHONY: all clean rebuild test directories copy_assets
