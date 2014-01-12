CC=arm-unknown-linux-gnueabi-g++
CFLAGS=-I/usr/local/include/urg_cpp -Wall
LFLAGS=-lpthread -lurg_cpp

VFHNavigation : main.cpp
	$(CC) $(CFLAGS) $(LFLAGS) main.cpp ../MTiG/libcmt.a ../drive/drive.cpp ../drive/gpio.cpp ../drive/pwm.cpp -o ~/Documents/BeagleBoneBlack/share/Program/VFHNavigation

myexample : myexample.cpp
	$(CC) $(CFLAGS) $(LFLAGS) myexample.cpp ../MTiG/libcmt.a -o myexample

clean:
	rm -f VFHNavigation
