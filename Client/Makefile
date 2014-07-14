CC=arm-unknown-linux-gnueabi-g++
CFLAGS=-I/usr/local/include -I/home/john/Documents/Program_YunTrooperII -O2 -Wall --std=c++11
LFLAGS=-lpthread -lurg_cpp -lGeographic

Client : main.cpp
	$(CC) $(CFLAGS) $(LFLAGS) main.cpp ../PathPoints/pathpoints.cpp ../communicator/communicator.cpp ../communicator/ringbuffer.cpp ../MTiG/libcmt.a ../drive/drive.cpp ../drive/gpio.cpp ../drive/pwm.cpp ../VFHPlus/vfhplus.cpp ../logfile/logfile.cpp ../Configuration/configuration.cpp -o ~/Documents/BeagleBoneBlack/share/Program/Client

clean :
	rm -f ~/Documents/BeagleBoneBlack/share/Program/Client
