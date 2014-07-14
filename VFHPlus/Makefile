CC=arm-unknown-linux-gnueabi-g++
CFLAGS=-I/usr/local/include -I/home/john/Documents/Program_YunTrooperII -Wall -O2
LFLAGS=-lpthread -lurg_cpp

VFHNavigation : main.cpp
	$(CC) $(CFLAGS) $(LFLAGS) main.cpp ../MTiG/libcmt.a ../drive/drive.cpp ../drive/gpio.cpp ../drive/pwm.cpp vfhplus.cpp ../Configuration/configuration.cpp ../logfile/logfile.cpp -o ~/Documents/BeagleBoneBlack/share/Program/VFHPlusNav

vfhplustest : vfhplustest.cpp vfhplus.cpp vfhplus.h
	$(CC) $(CFLAGS) $(LFLAGS) vfhplus.cpp vfhplustest.cpp -o vfhplustest

myexample : myexample.cpp
	$(CC) $(CFLAGS) $(LFLAGS) myexample.cpp ../MTiG/libcmt.a -o myexample

clean:
	rm -f VFHNavigation
