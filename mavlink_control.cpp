/** This example is public domain. */

/**
 * @file mavlink_control.cpp
 *
 * @brief The serial interface process
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include "system_ids.h"

//printf("hello");
// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// Default input arguments
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;

	// return code
	bool failure;

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------
	try
	{
		parse_commandline(argc, argv, uart_name, baudrate);
	}

	catch (int failure)
	{
		return failure;
	}

	// --------------------------------------------------------------------------
	//   START SERIAL PORT
	// --------------------------------------------------------------------------
	printf("OPEN PORT\n");

	try {
		open_serial(uart_name, baudrate);
	}

	catch (int failure)
	{
		return failure;
	}

	printf("\n");

	// --------------------------------------------------------------------------
	//   READ ONE MESSAGE
	// --------------------------------------------------------------------------
	printf("READ MAVLINK\n");
	read_message();
	printf("\n");

	// --------------------------------------------------------------------------
	//   SEND ONE MESSAGE
	// --------------------------------------------------------------------------
	printf("Write MAVLINK\n");
	write_message();
	printf("\n");

	// --------------------------------------------------------------------------
	//   CLOSE PORT
	// --------------------------------------------------------------------------
	printf("CLOSE PORT\n");
	close_serial();
	printf("\n");

	return 0;
}


// ------------------------------------------------------------------------------
//   Read Message
// ------------------------------------------------------------------------------
int
read_message()
{
	bool success;           // receive success flag
	bool received = false;  // receive only one message

	// Blocking wait for new data
	while (!received)
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = read_serial(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{
			// Handle Message ID
			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					// Decode Message
					mavlink_highres_imu_t imu;
					mavlink_msg_highres_imu_decode(&message, &imu);

					// Do something with the message
					printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
					printf("    time: %lu\n", imu.time_usec);
					printf("    acc  (NED):\t% f\t% f\t% f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
					printf("    gyro (NED):\t% f\t% f\t% f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
					printf("    mag  (NED):\t% f\t% f\t% f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
					printf("    baro: \t %f (mBar)\n"  , imu.abs_pressure);
					printf("    altitude: \t %f (m)\n" , imu.pressure_alt);
					printf("    temperature: \t %f C\n", imu.temperature );

					// Receive only one message
					received = true;
				}
				break;

			}
			// end: switch msgid

		}
		// end: if read message

	}

	return 0;
}


// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
write_message()
{

	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------
	mavlink_set_position_target_local_ned_t sp;
	sp.time_boot_ms     = 0;
	sp.type_mask        = 0;
	sp.target_system    = 1;
	sp.target_component = 1;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.x = 0.0f;
	sp.y = 0.0f;
	sp.z = 0.0f;
	sp.vx = 9000.0f;
	sp.vy = 0.0f;
	sp.vz = 0.0f;
	sp.afx = 0.0f;
	sp.afy = 9000.0f;
	sp.afz = 0.0f;
	sp.yaw = 0.0f;
	sp.yaw_rate = 0.0f;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(sysid, compid, &message, &sp);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------
	int len = write_serial(message);
	printf("Sent buffer of length %i\n",len);

	// Done!
	return 0;
}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";
	int i;

	// Read input arguments
	for (i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}




