PLATFORM = replica1

all: adventure.mon 

adventure.mon: adventure
	bintomon -1 -l 0x300 -r 0x300 adventure >adventure.mon

adventure: adventure.c 
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m adventure.map -t $(PLATFORM) adventure.c 

clean:
	$(RM) *.lst *.map *.mon *~




