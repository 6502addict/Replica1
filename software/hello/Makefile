PLATFORM = replica1

all: hello.mon

hello.mon: hello
	bintomon -1 -l 0x300 -r 0x300 hello >hello.mon

hello: hello.c  
	CC65_HOME=/usr/local/share/cc65 cl65 -O -vm -m hello.map -t $(PLATFORM) hello.c

clean:
	$(RM) *.lst *.map *.mon 


