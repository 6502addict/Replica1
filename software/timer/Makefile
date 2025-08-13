PLATFORM = replica1

all: timer.mon

timer.mon: timer
	bintomon -1 -l 0x300 -r 0x300 timer >timer.mon

timer: timer.c  ../lib/timer/timer.h ../lib/timer/timer.lib
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m timer.map -I. -I../lib/timer -t $(PLATFORM) timer.c ../lib/timer/timer.lib

clean:
	$(RM) *.lst *.map *.s



