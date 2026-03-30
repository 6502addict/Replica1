# Requires cc65 toolchain included
include ../../common.mk

# Default target
all: timer.lib timer.txt

# Build timer library from single source file
timer.lib: timer.o
	ar65 r timer.lib timer.o
	@echo "TIMER library created: timer.lib"

timer.s: timer.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -t $(TARGET) timer.c

timer.o: timer.s
	CC65_HOME=/usr/local/share/cc65 ca65 timer.s

timer.txt: timer.man
	nroff -man timer.man >timer.txt

# Clean build files
clean:
	rm -f *.o *.map *.s *~

# Install libraries to cc65 lib directory (optional)
install: timer.lib timer.h timer.man timer.md timer.txt
	cp timer.h   $(INCLUDE_DIR)
	cp timer.lib $(LIB_DIR)
	cp timer.md  $(DOC_DIR)
	cp timer.man $(MAN_DIR)
	cp timer.txt $(MAN_DIR)

.PHONY: all clean install
