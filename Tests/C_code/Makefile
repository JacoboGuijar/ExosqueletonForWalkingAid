# Makefile for BNO055 Quaternion Reader

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c11
LDFLAGS = -lm

# Target executable
TARGET = bno055_reader

# Source files
SRCS = main.c bno055.c
OBJS = $(SRCS:.c=.o)

# Header files
HEADERS = bno055.h

# Default target
all: $(TARGET)

# Link object files to create executable
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)
	@echo "Build successful! Run with: ./$(TARGET)"

# Compile source files to object files
%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -f $(OBJS) $(TARGET)
	@echo "Clean complete"

# Rebuild everything
rebuild: clean all

# Run the program (after building)
run: $(TARGET)
	./$(TARGET)

# Install target (optional - copies to /usr/local/bin)
install: $(TARGET)
	sudo cp $(TARGET) /usr/local/bin/
	@echo "Installed to /usr/local/bin/$(TARGET)"

# Uninstall target
uninstall:
	sudo rm -f /usr/local/bin/$(TARGET)
	@echo "Uninstalled from /usr/local/bin/$(TARGET)"

# Phony targets
.PHONY: all clean rebuild run install uninstall
