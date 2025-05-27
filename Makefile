#######################################
#   Makefile for Pure Data on Linux   #
#######################################

# === Compiler settings ===
# Use g++ as the C++ compiler
CXX = g++

# Compiler flags:
# -Wall: enable all warnings
# -Wextra: enable extra warnings
# -std=c++17: use the C++17 standard
# -fPIC: generate position-independent code (useful for shared libraries)
# -Iinclude: add 'include' directory to the header search path
# -MMD -MP: generate dependency files for header tracking
CXXFLAGS = -Wall -Wextra -std=c++17 -fPIC -Iinclude -MMD -MP

# === Directory layout ===
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

# === Pure Data sources ===
PD_SOURCES = \
	$(SRC_DIR)/adsr~.cpp \

PD_OBJECTS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(PD_SOURCES))
PD_TARGET = $(BIN_DIR)/adsr~.pd_linux

# === Dependency files ===
DEPS = $(PD_OBJECTS:.o=.d)

# === Default target ===
all: $(PD_TARGET)

# === Debug build ===
debug: CXXFLAGS += -O0 -g
debug: clean all

# === Release build ===
release: CXXFLAGS += -O3
release: clean all

# === Build rule for pd_linux external ===
$(PD_TARGET): $(PD_OBJECTS)
	@mkdir -p $(BIN_DIR)
	$(CXX) -shared -o $@ $^

# === Compile rule for .cpp to .o ===
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# === Clean rule ===
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# === Include dependency files ===
-include $(DEPS)

.PHONY: all clean debug release
