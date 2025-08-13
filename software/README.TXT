most of these programs with the replica 1 memory size set to 48K

to build these program you need to take the source code of cc65 patches for replica1

I modified the cc65 file to permit 48k ram


adventure /  animal /  commander / medieval  are AI generated games


hello the standard "hello world" to verify that the cc65 compiler is working...

I'm using the bintomon utility found on Jeff Tranter web site to convert the binary to a .mon

uploadable via a sendtext file of a terminal emulator


lib contains a series of libraries

spi     contains a spi libary to access the master spi controller located in $C200
        this library is quite trivial just access the spi master registers
	all the job is done by the vhdl layer

sdcard  contains a library to read and write a sdcard
        as it is the replica 1 is configured to use a seedstudio  v4.4 sdcard shield
	beware that this one is compatible 3.3v many others would destroy you DE10-lite
	simply because they have extra 5v signals (specially adafruit datalogger)
	when testing use empty sdcard to avoid loosing data


timer   contains a library to count the time spend or create some delay


w25qxx  is a library built to access w25qXX spi flash from winbond
        same thing use empty spi flash to avoid loosing data


tests programs:

sdcard contains a program verifying that the spi hardware and spi + sdcard library are working
       this program trash the sdcard inserted...
 
timer  simple program to test the execution speed of a function (see timer.c)
