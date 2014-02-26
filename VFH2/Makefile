CC=arm-unknown-linux-gnueabi-g++
CFLAGS=-I/usr/local/include/urg_cpp -I/home/john/Documents/Program_YunTrooperII -Wall
LFLAGS=-lpthread -lurg_cpp

VFHNavigation : main.cpp
	$(CC) $(CFLAGS) $(LFLAGS) main.cpp ../MTiG/libcmt.a ../drive/drive.cpp ../drive/gpio.cpp ../drive/pwm.cpp vfh2.cpp -o ~/Documents/BeagleBoneBlack/share/Program/VFHNavigationNew

myexample : myexample.cpp
	$(CC) $(CFLAGS) $(LFLAGS) myexample.cpp ../MTiG/libcmt.a -o myexample

clean:
	rm -f VFHNavigation
