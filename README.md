C-UART Interface Example
========================

This is a simple MAVLink to UART interface example for *nix systems that can allow communication between Pixhawk and an offboard computer.

This example will recieve one MAVlink message and send one MAVlink message.


Building
========

```
$ cd c_uart_interface_example/
$ make
```

Hardware Setup
=========

Connect the USB programming cable to your Pixhawk.  

If you want to be able to interact with this example in Pixhawk's NuttX shell, you'll need a Telemetry Radio or an FTDI developer's cable.  See the Exploration section below for more detail.

Note: Serial 5's receive pin is occupied by a second NuttX shell and can't be used to receive data without reconfiguration.

Also Note: Using a UART (serial) connection should be preferred over using the USB port for flying systems.  The reason being that the driver for the USB port is much more complicated, so the UART is a much more trusted port for flight-critical functions.  To learn how this works though the USB port will be fine and instructive.

Execution
=========

1. Login to Pixhawk's shell
-----------------------

```
screen /dev/ttyACM0 57600 8N1
<press enter>
```

You have to pick a port name, if the above example doesn't work, try searching for it with 
```

$ ls /dev/ttyACM* 
$ ls /dev/ttyUSB*
```

Alternatively, plug in Pixhawk USB cable again and issue the command:
```
$ dmesg
```
The device described at the bottom of dmesg's output will be the port on which the Pixhawk is mounted. 

The Pixhawk USB port will show up on a ```ttyACM*```, an FTDI cable will show up on a ```ttyUSB*```.


2. Start a mavlink session on Pixhawk's USB port
-----------------------

```
nsh> mavlink start -d /dev/ttyACM0
```

Pixhawk will start dumping machine data to the shell.

Exit screen with the key sequence: ```Ctrl+A , K, Y```

3. Run the Example Executable
-----------------------------

```
$ cd c_uart_interface_example/
$ ./mavlink_control -d /dev/ttyACM0
```

To stop the program, use the key sequence ```Ctrl-C```.

Here's an example output:

```
OPEN PORT
Connected to /dev/ttyUSB0 with 57600 baud, 8 data bits, no parity, 1 stop bit (8N1)


READ MAVLINK
Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)
    time: 6656240526
    acc  (NED):	 0.021770	-0.038049	-9.857707 (m/s^2)
    gyro (NED):	-0.000756	-0.001846	-0.006603 (rad/s)
    mag  (NED):	 0.174230	-0.145237	 0.233960 (Ga)
    baro: 	 1018.039978 (mBar)
    altitude: 	 -39.796490 (m)
    temperature: 	 42.200001 C

Start Off-Board Mode
Write Off-Board Commands
Sent buffer of length 61
Sent buffer of length 61
Sent buffer of length 61
Sent buffer of length 61
Sent buffer of length 61
Sent buffer of length 61

^CTerminating at user's request

Sent buffer of length 61
Stop Off-Board Mode

CLOSE PORT
Port closed
```

Exploration
===========

There are a few things to explore past this example.

First you can connect via a Telemetry Radio on Telem 1 or 2, or via an FTDI on Telem 2 or Serial 4 
(https://pixhawk.org/dev/wiring).  Note: Serial 5's receive pin is occupied by a second NuttX shell and can't be used to receive data without reconfiguration.

With this you'll be able to start a second port for communcation, and leave the USB port available for viewing prints in the NuttX shell.  

For steps 2 and 3 from the above tutorial, you'll use a different port.  On the off-board computer side, the port might now be ```/dev/ttyUSB0```.  On the Pixhawk side, here the port mappings are in the table below.

| PX4 UART | NuttX UART |
|----------|------------|
| Telem 1  | /dev/ttyS1 |
| Telem 2  | /dev/ttyS2 |
| Serial 4 | /dev/ttyS6 |

Now add a print statement in the Pixhawk Firmware to see received messages.  Build and upload this to Pixhawk.

```
[Firmware/src/modules/mavlink/mavlink_receiver.cpp : line 1351]
/* if read failed, this loop won't execute */
for (ssize_t i = 0; i < nread; i++) {
	if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &status)) {

		/* --- REPORT HANDLING OF MESSAGE --- */
		printf("\n");
		printf("HANDLE MESSAGE\n");
		printf("MSGID:%i\n",msg.msgid);

		/* handle generic messages and commands */
		handle_message(&msg);
```

Screen into the NuttX shell and start a mavlink session like in the example above.  

On the off-board side, in another terminal run the ```c_uart_interface_example``` executable. You should see output in the NuttX shell similar to this:

```
HANDLE MESSAGE
MSGID:76

HANDLE MESSAGE
MSGID:84

(...)

HANDLE MESSAGE
MSGID:84

HANDLE MESSAGE
MSGID:76
```

Past this, you can:
- Modify the received message data type
- Modify the sent message data type





