# Requires cc65 toolchain installed

CC65_HOME=/usr/local/share/cc65 

# Default target
all: w25qxx.lib  flash-id.mon flash-erase.mon flash-dump.mon

flash-id.mon: flash-id
	bintomon -1 -l 0x300 -r 0x300 flash-id >flash-id.mon

flash-erase.mon: flash-erase
	bintomon -1 -l 0x300 -r 0x300 flash-erase >flash-erase.mon

flash-dump.mon: flash-dump
	bintomon -1 -l 0x300 -r 0x300 flash-dump >flash-dump.mon

# Build SPI library
w25qxx.lib: w25qxx.o 
	ar65 r w25qxx.lib w25qxx.o
	@echo "W25QXX library created: w25qxx.lib"

w25qxx.s: w25qxx.c ../spi/spi.h w25qxx.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O  -t replica1 -I. -I../spi w25qxx.c ../spi/spi.lib

flash-id: flash-id.c ../spi/spi.h w25qxx.h w25qxx.lib
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m flash-id.map -I. -I../lib/spi -t replica1 flash-id.c w25qxx.lib ../spi/spi.lib

flash-erase: flash-erase.c ../spi/spi.h w25qxx.h w25qxx.lib
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m flash-erase.map -I. -I../lib/spi -t replica1 flash-erase.c w25qxx.lib ../spi/spi.lib

flash-dump: flash-dump.c ../spi/spi.h w25qxx.h w25qxx.lib
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m flash-dump.map -I. -I../lib/spi -t replica1 flash-dump.c w25qxx.lib ../spi/spi.lib

w25qxx.o: w25qxx.s
	CC65_HOME=/usr/local/share/cc65 ca65  w25qxx.s


# Clean build files
clean:
	rm -f *.o *.map *~ *.s flash-it flash-erase flash-dump *.mon

# Install libraries to cc65 lib directory (optional)
install: $(SPI_LIB) $(SDCARD_LIB)
	cp $(SPI_LIB) $(SDCARD_LIB) $(CC65_HOME)/lib/
	cp include/*.h $(CC65_HOME)/include/

# Create directory structure
setup:
	mkdir -p src include examples

.PHONY: all clean install setup




