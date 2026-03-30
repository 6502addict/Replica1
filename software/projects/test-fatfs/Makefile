# Requires cc65 toolchain installed

include ../common.mk

LIBS = $(LIB_DIR)/fatfs.lib $(LIB_DIR)/sdcard.lib $(LIB_DIR)/spi.lib $(LIB_DIR)/timer.lib

# Default target
all: test-fatfs.mon

test-fatfs.mon: test-fatfs.bin
	bintomon -1 -l 0x300 -r 0x300 test-fatfs.bin >test-fatfs.mon

test-fatfs.bin: test-fatfs.c $(LIBS) 
	CC65_HOME=$(CC65_HOME) cl65 -O -vm -m test-fatfs.map -o test-fatfs.bin  $(INCLUDES) -t $(TARGET) test-fatfs.c $(LIBS)

# Clean build files
clean:
	rm -f *.o *.map *.s *~ *.bin

# Install libraries to cc65 lib directory (optional)
install: test-fatfs.mon
	cp test-fatfs.mon  $(TESTS)

.PHONY: all clean install




