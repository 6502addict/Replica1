# Requires cc65 toolchain installed

include ../../common.mk

# Default target
all: fatfs.lib 

fatfs.lib: ff.o  diskio.o
	ar65 r fatfs.lib ff.o diskio.o
	@echo "FATFS library created: fatfs.lib"

ff.s: ff.c ff.h ffconf.h
	CC65_HOME=$(CC65_HOME) cc65 -O $(INCLUDES)  -t $(TARGET) ff.c

diskio.s: diskio.c 
	CC65_HOME=$(CC65_HOME) cc65 -O $(INCLUDES)  -t $(TARGET) diskio.c

ff.o: ff.s
	CC65_HOME=$(CC65_HOME) ca65  ff.s

diskio.o: diskio.s
	CC65_HOME=$(CC65_HOME) ca65  diskio.s

# Clean build files
clean:
	rm -f *.o *.map *.s *~ *.bin

# Install libraries to cc65 lib directory (optional)
install: ff.h ffconf.h fatfs.lib
	cp ff.h       $(INCLUDE_DIR)
	cp ffconf.h   $(INCLUDE_DIR)
	cp fatfs.lib  $(LIB_DIR)

# Create directory structure
setup:
	mkdir -p src include examples

.PHONY: all clean install setup




