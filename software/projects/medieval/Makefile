PLATFORM = max6502

include ../common.mk

all: medieval.mon 

medieval.mon: medieval
	bintomon -1 -l 0x300 -r 0x300 medieval >medieval.mon

medieval: medieval.c 
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m medieval.map -t $(PLATFORM) medieval.c 

clean:
	$(RM) *.lst *.map *.mon *~


# Install libraries to cc65 lib directory (optional)
install: medieval.mon
	cp medieval.mon  $(TESTS)

.PHONY: all clean install



