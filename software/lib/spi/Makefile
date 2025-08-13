# Building cc65 Libraries for SPI and SD Card
# Requires cc65 toolchain installed

# Compiler and tools
CC = cc65
AS = ca65
AR = ar65
TARGET = replica1
CC65_HOME=/usr/local/share/cc65 


# Default target
all: spi.lib

# Build SPI library
spi.lib: spi-init.o spi-transfer.o
	ar65 r spi.lib spi-init.o spi-transfer.o
	@echo "SPI library created: spi.lib"

spi-init.s: spi-init.c
	CC65_HOME=/usr/local/share/cc65 cc65 -O  -t replica1 spi-init.c 

spi-transfer.s: spi-transfer.c
	CC65_HOME=/usr/local/share/cc65 cc65 -O -t replica1 spi-transfer.c

spi-init.o: spi-init.s
	CC65_HOME=/usr/local/share/cc65 ca65  spi-init.s

spi-transfer.o: spi-transfer.s
	CC65_HOME=/usr/local/share/cc65 ca65  spi-transfer.s


# Clean build files
clean:
	rm -f *.o *.map *~ *.s

# Install libraries to cc65 lib directory (optional)
install: $(SPI_LIB) $(SDCARD_LIB)
	cp $(SPI_LIB) $(SDCARD_LIB) $(CC65_HOME)/lib/
	cp include/*.h $(CC65_HOME)/include/

# Create directory structure
setup:
	mkdir -p src include examples

.PHONY: all clean install setup




