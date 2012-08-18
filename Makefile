all: mavlink_serial

mavlink_serial: mavlink_serial.o
	g++ mavlink_serial.o -o mavlink_serial

mavlink_serial.o: mavlink_serial.cpp
	g++ -I mavlink/include/mavlink/v1.0 -c mavlink_serial.cpp

clean:
	rm -rf *o mavlink_serial