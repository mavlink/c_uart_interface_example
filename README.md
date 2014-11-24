C-UART Interface Example
========================

This is a simple MAVLink to UART interface example for *nix systems that can allow communication between Pixhawk and an offboard computer.

This example will recieve one MAVlink message and send one MAVlink message.


Building
========

```
$ cd c_uart_interface_example/build/
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
$ cd c_uart_interface_example/build
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

First you can connect via a Telemetry Radio on Telem 1 or 2, or via an FTDI on the Serial 4/5 port 
(https://pixhawk.org/dev/wiring).

With this you'll be able to start this port for communcation, and leave the USB port available for viewing prints in the NuttX shell.  You'll use a different port in steps 2 and 3 above.

On the off-board computer side, the port might now be ```/dev/ttyUSB0```

On the Pixhawk side, here are the port mappings

| PX4 UART | NuttX UART |
|----------|------------|
| Telem 1  | /dev/ttyS1 |
| Telem 2  | /dev/ttyS2 |
| Serial 4 | /dev/ttyS6 |
| Serial 5 | /dev/ttyS5 |

Now add a print statement in the Pixhawk Firmware to see received messages.  Build and upload this to Pixhawk.

```
[Firmware/src/modules/mavlink/mavlink_receiver.cpp : line 1351]
/* if read failed, this loop won't execute */
for (ssize_t i = 0; i < nread; i++) {
	if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &status)) {

		/* --- REPORT HANDLING OF MESSAGE --- */
		printf("HANDLE MESSAGE\n");
		printf("MSGID:%i\n",msg.msgid);

		/* handle generic messages and commands */
		handle_message(&msg);
```

Screen into the NuttX shell, and in another terminal run the ```c_uart_interface_example``` executable. You should see output in the NuttX shell similar to this:

```
HANDLE MESSAGE
MSGID:84
```

Past this, you can:
- Modify the received message data type
- Modify the sent message data type





