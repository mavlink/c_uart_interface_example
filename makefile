all: mavlink_control

mavlink_control: mavlink_control.cpp
	g++ -I mavlink/include/mavlink/v1.0 mavlink_control.cpp serial_port.cpp offboard_setup.cpp -o mavlink_control

clean:
	 rm -rf *o mavlink_control
