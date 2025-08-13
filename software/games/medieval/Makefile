PLATFORM = replica1

all: medieval.mon 

medieval.mon: medieval
	bintomon -1 -l 0x300 -r 0x300 medieval >medieval.mon

medieval: medieval.c 
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m medieval.map -t $(PLATFORM) medieval.c 

clean:
	$(RM) *.lst *.map *.mon *~




