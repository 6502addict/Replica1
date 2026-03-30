
include ../common.mk

LIBS = $(LIB_DIR)/timer.lib

all: hello.mon

hello.mon: hello.bin
	bintomon -1 -l 0x300 -r 0x300 hello.bin >hello.mon

hello.bin: hello.c $(INCLUDE_DIR)/timer.h $(LIBS)
	CC65_HOME=$(CC65_HOME) cl65 -O -vm -m hello.map $(INCLUDES) -o hello.bin -t $(TARGET) hello.c $(LIBS)

clean:
	$(RM) *.lst *.map *.mon *.bin *~

install: hello.mon
	cp hello.mon  $(TESTS)

.PHONY: all clean install

