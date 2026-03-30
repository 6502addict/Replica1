# Requires cc65 toolchain installed

include ../common.mk

LIBS = $(LIB_DIR)/fatfs.lib $(LIB_DIR)/sdcard.lib $(LIB_DIR)/spi.lib $(LIB_DIR)/timer.lib

# Default target
all: dskbrowser.mon

dskbrowser.mon: dskbrowser.bin
	bintomon -1 -l 0x300 -r 0x300 dskbrowser.bin >dskbrowser.mon

dskbrowser.bin: dskbrowser.c $(LIBS) 
	CC65_HOME=$(CC65_HOME) cl65 -O -vm -m dskbrowser.map -o dskbrowser.bin  $(INCLUDES) -t $(TARGET) dskbrowser.c $(LIBS)

# Clean build files
clean:
	rm -f *.o *.map *.s *~ *.bin

# Install libraries to cc65 lib directory (optional)
install: dskbrowser.mon
	cp dskbrowser.mon  $(TESTS)

.PHONY: all clean install




