PLATFORM = replica1

all: sdcard.mon 

sdcard.mon: sdcard
	bintomon -1 -l 0x300 -r 0x300 sdcard >sdcard.mon

sdcard: sdcard.c ../lib/spi/spi.h ../lib/spi/spi.lib ../lib/sdcard/sdcard.h ../lib/sdcard/sdcard.lib
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m sdcard.map -I. -I../lib/spi -I../lib/sdcard -t $(PLATFORM) sdcard.c ../lib/sdcard/sdcard.lib ../lib/spi/spi.lib

clean:
	$(RM) *.lst *.map *.mon 


