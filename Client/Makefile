CC=arm-unknown-linux-gnueabi-g++
CFLAGS=-I/usr/local/include/urg_cpp -I/home/john/Documents/Program_YunTrooperII -Wall --std=c++11
LFLAGS=-lpthread -lurg_cpp

Client : main.cpp
	$(CC) $(CFLAGS) $(LFLAGS) main.cpp ../PathPoints/pathpoints.cpp ../communicator/communicator.cpp ../communicator/ringbuffer.cpp ../MTiG/libcmt.a ../drive/drive.cpp ../drive/gpio.cpp ../drive/pwm.cpp ../VFH2/vfh2.cpp ../logfile/logfile.cpp -o ~/Documents/BeagleBoneBlack/share/Program/Client

clean:
	rm -f ~/Documents/BeagleBoneBlack/share/Program/Client
