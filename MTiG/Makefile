CC=arm-unknown-linux-gnueabi-g++
CFLAGS=-I. -Wall
LFLAGS=-lrt -lpthread
DEPS = cmt1.h 
OBJSTATIC = cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o

all: static example example1s example2s example3s myexample

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

example: $(OBJSTATIC) example_linux.cpp
	$(CC) $(CFLAGS) $(LFLAGS) $(OBJSTATIC) -lncurses example_linux.cpp -o example

example1s: $(OBJSTATIC) example1s_mt_linux.cpp
	$(CC) $(CFLAGS) $(LFLAGS) $(OBJSTATIC) example1s_mt_linux.cpp -o example1s

example2s: $(OBJSTATIC) example2s_mt_linux.cpp
	$(CC) $(CFLAGS) $(LFLAGS) $(OBJSTATIC) example2s_mt_linux.cpp -o example2s

example3s: $(OBJSTATIC) example3s_mt_linux.cpp
	$(CC) $(CFLAGS) $(LFLAGS) $(OBJSTATIC) example3s_mt_linux.cpp -o example3s

myexample: $(OBJSTATIC) myexample.cpp
	$(CC) $(CFLAGS) $(LFLAGS) $(OBJSTATIC) myexample.cpp -o myexample

GPSReceive: $(OBJSTATIC) GPSReceive.cpp
	$(CC) $(CFLAGS) $(LFLAGS) $(OBJSTATIC) GPSReceive.cpp -o GPSReceive

static: $(OBJSTATIC)
	ar rcs libcmt.a $^

clean:
	rm -f example
	rm -f example1s
	rm -f example2s
	rm -f example3s
	rm -f myexample
	rm -f *.o
	rm -f libcmt.a
	rm -f libcmt.so
