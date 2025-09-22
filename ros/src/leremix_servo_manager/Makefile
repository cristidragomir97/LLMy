# Simple Makefile for standalone motor tests
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -Iinclude
SRCDIR = src
OBJDIR = build

# Source files
MOTOR_MANAGER_SRC = $(SRCDIR)/MotorManager.cpp
SPIN_BRAKE_TEST_SRC = $(SRCDIR)/spin_brake_test.cpp
PING_TEST_SRC = $(SRCDIR)/ping_test.cpp

# Object files
MOTOR_MANAGER_OBJ = $(OBJDIR)/MotorManager.o
SPIN_BRAKE_TEST_OBJ = $(OBJDIR)/spin_brake_test.o
PING_TEST_OBJ = $(OBJDIR)/ping_test.o

# Executables
SPIN_BRAKE_TEST = spin_brake_test
PING_TEST = ping_test

.PHONY: all clean spin_brake_test ping_test

all: $(SPIN_BRAKE_TEST) $(PING_TEST)

# Create build directory
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Compile MotorManager
$(MOTOR_MANAGER_OBJ): $(MOTOR_MANAGER_SRC) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Compile spin_brake_test
$(SPIN_BRAKE_TEST_OBJ): $(SPIN_BRAKE_TEST_SRC) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Compile ping_test  
$(PING_TEST_OBJ): $(PING_TEST_SRC) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Link spin_brake_test
$(SPIN_BRAKE_TEST): $(MOTOR_MANAGER_OBJ) $(SPIN_BRAKE_TEST_OBJ)
	$(CXX) $^ -o $@

# Link ping_test
$(PING_TEST): $(MOTOR_MANAGER_OBJ) $(PING_TEST_OBJ)
	$(CXX) $^ -o $@

# Individual targets
spin_brake_test: $(SPIN_BRAKE_TEST)

ping_test: $(PING_TEST)

clean:
	rm -rf $(OBJDIR) $(SPIN_BRAKE_TEST) $(PING_TEST)

install: $(SPIN_BRAKE_TEST)
	cp $(SPIN_BRAKE_TEST) /usr/local/bin/

.SUFFIXES: .cpp .o