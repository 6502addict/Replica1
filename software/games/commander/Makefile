PLATFORM = replica1

all: commander.mon 

commander.mon: commander
	bintomon -1 -l 0x300 -r 0x300 commander >commander.mon

commander: commander.c 
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m commander.map -t $(PLATFORM) commander.c 

clean:
	$(RM) *.lst *.map *.mon *~




