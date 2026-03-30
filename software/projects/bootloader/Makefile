# Requires cc65 toolchain installed

include ../common.mk

CMD = bootloader

LIBS = $(LIB_DIR)/fatfs.lib $(LIB_DIR)/sdcard.lib $(LIB_DIR)/spi.lib $(LIB_DIR)/timer.lib

# Default target
all: $(CMD).mon

$(CMD).mon: $(CMD).bin
	bintomon -1 -l 0x300 -r 0x300 $(CMD).bin >$(CMD).mon

$(CMD).bin: $(CMD).c $(LIBS) 
	CC65_HOME=$(CC65_HOME) cl65 -O -vm -m $(CMD).map -o $(CMD).bin  $(INCLUDES) -t $(TARGET) $(CMD).c $(LIBS)

# Clean build files
clean:
	rm -f *.o *.map *.s *~ *.bin

.PHONY: all clean 




