# Compiler settings
CXX := clang++
CXXFLAGS := -Wall -Wextra -std=c++14 -I./include -ffast-math -mrecip -march=native \
            -I/opt/homebrew/include
LDFLAGS := -L/opt/homebrew/lib -lSDL2 -lSDL2_ttf

# Directory structure
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
TEST_DIR := test

# Find all source files
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)

# Main target
TARGET := $(BUILD_DIR)/simulator

# Default target
all: directories $(TARGET)

# Create necessary directories
directories:
	@mkdir -p $(BUILD_DIR)

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

# Clean and rebuild
rebuild: clean all

.PHONY: all clean rebuild test directories
