PLATFORM = replica1

all: animal.mon 

animal.mon: animal
	bintomon -1 -l 0x300 -r 0x300 animal >animal.mon

animal: animal.c 
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m animal.map -t $(PLATFORM) animal.c 

clean:
	$(RM) *.lst *.map *.mon *~




