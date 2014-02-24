CROSS_CC=arm-unknown-linux-gnueabi-g++
CC=g++
CFLAGS=-I/home/john/Documents/Program_YunTrooperII -Wall
LFLAGS=-lpthread

Host : main.cpp
	$(CC) $(CFLAGS) $(LFLAGS) main.cpp ../communicator/communicator.cpp ../communicator/ringbuffer.cpp ../joystick/controller.cpp -o ./Host
clean:
	rm -rf ./Host
