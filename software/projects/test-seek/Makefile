# Requires cc65 toolchain installed

include ../common.mk

LIBS = $(LIB_DIR)/fatfs.lib $(LIB_DIR)/sdcard.lib $(LIB_DIR)/spi.lib $(LIB_DIR)/timer.lib

# Default target
all: test-seek.mon

test-seek.mon: test-seek.bin
	bintomon -1 -l 0x300 -r 0x300 test-seek.bin >test-seek.mon

test-seek.bin: test-seek.c $(LIBS) 
	CC65_HOME=$(CC65_HOME) cl65 -O -vm -m test-seek.map -o test-seek.bin  $(INCLUDES) -t $(TARGET) test-seek.c $(LIBS)

# Clean build files
clean:
	rm -f *.o *.map *.s *~ *.bin

# Install libraries to cc65 lib directory (optional)
install: test-seek.mon
	cp test-seek.mon  $(TESTS)

.PHONY: all clean install




