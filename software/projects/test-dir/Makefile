# Requires cc65 toolchain installed

include ../common.mk

LIBS = $(LIB_DIR)/fatfs.lib $(LIB_DIR)/sdcard.lib $(LIB_DIR)/spi.lib $(LIB_DIR)/timer.lib

# Default target
all: test-dir.mon

test-dir.mon: test-dir.bin
	bintomon -1 -l 0x300 -r 0x300 test-dir.bin >test-dir.mon

test-dir.bin: test-dir.c $(LIBS) 
	CC65_HOME=$(CC65_HOME) cl65 -O -vm -m test-dir.map -o test-dir.bin  $(INCLUDES) -t $(TARGET) test-dir.c $(LIBS)

# Clean build files
clean:
	rm -f *.o *.map *.s *~ *.bin

install: test-dir.mon
	cp test-dir.mon  $(TESTS)

.PHONY: all clean install



