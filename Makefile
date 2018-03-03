CXX = clang++
SDL = -framework SDL2
# If your compiler is a bit older you may need to change -std=c++11 to -std=c++0x
EXE = simulator
SRC_DIR := .
OBJ_DIR := ./obj
SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))
LDFLAGS = $(SDL)
CXXFLAGS = -Wall -c -std=c++11

all: $(EXE)

$(EXE): $(OBJ_FILES)
	g++ $(LDFLAGS) -o $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	g++ $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f *.o ; rm -f $(EXE) ; rm -f obj/*
