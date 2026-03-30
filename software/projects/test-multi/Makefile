# Requires cc65 toolchain installed

include ../common.mk

LIBS = $(LIB_DIR)/fatfs.lib $(LIB_DIR)/sdcard.lib $(LIB_DIR)/spi.lib $(LIB_DIR)/timer.lib

# Default target
all: test-multi.mon

test-multi.mon: test-multi.bin
	bintomon -1 -l 0x300 -r 0x300 test-multi.bin >test-multi.mon

test-multi.bin: test-multi.c $(LIBS) 
	CC65_HOME=$(CC65_HOME) cl65 -O -vm -m test-multi.map -o test-multi.bin  $(INCLUDES) -t $(TARGET) test-multi.c $(LIBS)

# Clean build files
clean:
	rm -f *.o *.map *.s *~ *.bin

# Install libraries to cc65 lib directory (optional)
install: test-multi.mon
	cp test-multi.mon  $(TESTS)

.PHONY: all clean install




