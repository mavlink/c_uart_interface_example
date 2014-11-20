c_uart_interface_example
========================

This is a simple MAVLink to UART interface example for *nix systems that can allow communication between Pixhawk and an offboard computer.

This example will recieve one MAVlink message and send one MAVlink message.


Building
========

```
$ cd c_uart_interface_example/Release/
$ make
```

Harware Setup
=========

Connect the USB programming cable to your Pixhawk.  

If you want to be able to interact with the Pixhawk's Nuttix shell, you'll need an FTDI developer's cable, connected to Serial 4/5.

https://pixhawk.org/dev/wiring


Execution
=========

1. Login to Pixhawk's shell
-----------------------

```
screen /dev/ttyACM0 57600 8N1
<press enter>
```

2. Start a mavlink session on Pixhawk's USB port
-----------------------

```
nsh> mavlink start -d /dev/ttyACM0
```

Pixhawk will start dumping machine data to the shell.
Exit screen with the key sequence: ```Ctrl+A , K, Y```

3. Run the Example executable.
-----------------------------

```
$ cd c_uart_interface_example/Release
$ ./c_uart_interface_example -d /dev/ttyACM0
```

You have to pick a port name, if the above exampled doesn't work, try searching for it with 
```
$ ls /dev/ttyACM*
$ ls /dev/ttyUSB*
```

Here's an example output

```
OPEN PORT
Connected to /dev/ttyACM0 with 57600 baud, 8 data bits, no parity, 1 stop bit (8N1)
READ MAVLINK
Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)
	 time: 22146030
	 acc  (NED):	 0.452423	-0.052777	-9.760157 (m/s^2)
	 gyro (NED):	-0.004391	-0.002016	-0.004369 (rad/s)
	 mag  (NED):	-0.215094	 0.394682	 1.033930 (Ga)
	 baro: 	 1018.009949 (mBar)
	 altitude: 	 -39.547714 (m)
	 temperature: 	 43.639999 C

SEND MAVLINK
Sent buffer of length 61
CLOSE PORT
Port closed
```

Exploration
===========

There are a few things to explore past this example.

1. Connect via an FTDI cable on the Telem 2 or Serial 4/5 port 
(https://pixhawk.org/dev/wiring).
On the off-board computer side, the port might be ```/dev/ttyUSB0```
If using Serial 4/5 on the Pixhawk side, the port will be ```/dev/ttyS5```

2. Modify the received message data type
3. Modify the sent message data type





