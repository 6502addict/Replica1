# Requires cc65 toolchain installed

include ../common.mk

LIBS = $(LIB_DIR)/sdcard.lib  $(LIB_DIR)/spi.lib $(LIB_DIR)/timer.lib

# Default target
all: spi-speed.mon

spi-speed.mon: spi-speed.bin
	bintomon -1 -l 0x300 -r 0x300 spi-speed.bin >spi-speed.mon

spi-speed.bin: spi-speed.c $(LIBS) 
	CC65_HOME=$(CC65_HOME) cl65 -O -vm -m spi-speed.map -o spi-speed.bin  $(INCLUDES) -t $(TARGET) spi-speed.c $(LIBS)

# Clean build files
clean:
	rm -f *.o *.map *.s *~ *.bin

# Install libraries to cc65 lib directory (optional)
install: spi-speed.mon
	cp spi-speed.mon  $(TESTS)

.PHONY: all clean install




