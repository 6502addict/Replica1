# Building cc65 Libraries for SPI and SD Card

# Compiler and tools
include ../../common.mk

# Default target
all: spi.lib spi.txt

# Build SPI library
spi.lib: spi.o
	ar65 r spi.lib spi.o
	@echo "SPI library created: spi.lib"

spi.s: spi.c
	CC65_HOME=/usr/local/share/cc65 cc65 -O $(INCLUDES)  -t $(TARGET) spi.c 

spi.o: spi.s
	CC65_HOME=/usr/local/share/cc65 ca65  spi.s

spi.txt: spi.man
	nroff -man spi.man >spi.txt

# Clean build files
clean:
	rm -f *.o *.map *~ *.s

# Install libraries to cc65 lib directory (optional)
install: spi.h spi.lib spi.md spi.man spi.txt
	cp spi.h    $(INCLUDE_DIR)
	cp spi.lib  $(LIB_DIR)
	cp spi.man  $(MAN_DIR)
	cp spi.txt  $(MAN_DIR)
	cp spi.md   $(DOC_DIR)

# Create directory structure
setup:
	mkdir -p src include examples

.PHONY: all clean install setup




