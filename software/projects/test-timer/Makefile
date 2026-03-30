# Requires cc65 toolchain installed

include ../common.mk

LIBS = $(LIB_DIR)/fatfs.lib $(LIB_DIR)/sdcard.lib $(LIB_DIR)/spi.lib $(LIB_DIR)/timer.lib

# Default target
all: test-timer.mon

test-timer.mon: test-timer.bin
	bintomon -1 -l 0x300 -r 0x300 test-timer.bin >test-timer.mon

test-timer.bin: test-timer.c $(LIBS) 
	CC65_HOME=$(CC65_HOME) cl65 -O -vm -m test-timer.map -o test-timer.bin  $(INCLUDES) -t $(TARGET) test-timer.c $(LIBS)

# Clean build files
clean:
	rm -f *.o *.map *.s *~ *.bin

# Install libraries to cc65 lib directory (optional)
install: test-timer.mon
	cp test-timer.mon  $(TESTS)

.PHONY: all clean install




