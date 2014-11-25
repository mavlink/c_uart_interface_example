all: mavlink_control

mavlink_control: mavlink_control.o
	g++ mavlink_control.o -o mavlink_control

mavlink_control.o: mavlink_control.cpp
	g++ -I mavlink/include/mavlink/v1.0 mavlink_control.cpp serial_port.cpp 

clean: rm -rf *o mavlink_control
