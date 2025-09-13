# Requires cc65 toolchain installed

# Compiler and tools
CC = cc65
AS = ca65
AR = ar65
TARGET = replica1
CC65_HOME=/usr/local/share/cc65 


# Default target
all: fatfs.lib test-fatfs.mon

test-fatfs.mon: test-fatfs.bin
	bintomon -1 -l 0x300 -r 0x300 test-fatfs.bin >test-fatfs.mon

# Build SPI library
fatfs.lib: ff.o  diskio.o
	ar65 r fatfs.lib ff.o diskio.o
	@echo "FATFS library created: fatfs.lib"

ff.s: ff.c ff.h ffconf.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t replica1 ff.c

diskio.s: diskio.c 
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../sdcard -I../spi  -t replica1 diskio.c

ff.o: ff.s
	CC65_HOME=/usr/local/share/cc65 ca65  ff.s

diskio.o: diskio.s
	CC65_HOME=/usr/local/share/cc65 ca65  diskio.s

test-fatfs.bin: test-fatfs.c ../sdcard/sdcard.lib  fatfs.lib ../sdcard/sdcard.lib ../spi/spi.lib
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m test-fatfs.map -o test-fatfs.bin -I. -I../lib/spi -I../lib/sdcard -t replica1 test-fatfs.c fatfs.lib ../sdcard/sdcard.lib ../spi/spi.lib

# Clean build files
clean:
	rm -f *.o *.map *.s

# Install libraries to cc65 lib directory (optional)
install: $(SPI_LIB) $(SDCARD_LIB)
	cp $(SPI_LIB) $(SDCARD_LIB) $(CC65_HOME)/lib/
	cp include/*.h $(CC65_HOME)/include/

# Create directory structure
setup:
	mkdir -p src include examples

.PHONY: all clean install setup




