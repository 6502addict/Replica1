# Requires cc65 toolchain installed

include ../../common.mk

# Default target
all: sdcard.lib

# Build SPI library
sdcard.lib: sdcard.o  
	ar65 r sdcard.lib sdcard.o
	@echo "SDCARD library created: sdcard.lib"


sdcard.s: sdcard.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. $(INCLUDES) -t $(TARGET) sdcard.c

sdcard.o: sdcard.s
	CC65_HOME=/usr/local/share/cc65 ca65  sdcard.s


# Clean build files
clean:
	rm -f *.o *.map  *~ *.s

# Install libraries to cc65 lib directory (optional)
install: sdcard.h sdcard.lib
	cp sdcard.h $(INCLUDE_DIR)
	cp sdcard.lib $(LIB_DIR)

# Create directory structure
setup:
	mkdir -p src include examples

.PHONY: all clean install setup




