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
 * @author Jaycee Lock, <jaycee.lock@gmail.com>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include "serial_port.h"
#include "system_ids.h"
#include "offboard_setup.h"

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// Default input arguments
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	try
	{
		parse_commandline(argc, argv, uart_name, baudrate);
	}

	catch ( int failure )
	{
		return failure;
	}


	// --------------------------------------------------------------------------
	//   START SERIAL PORT
	// --------------------------------------------------------------------------

	printf("OPEN PORT\n");

	try
	{
		open_serial(uart_name, baudrate);
	}

	catch ( int failure )
	{
		return failure;
	}

	// Set up program termination
	signal(SIGINT, quit_handler);

	printf("\n");


	// --------------------------------------------------------------------------
	//   READ ONE MESSAGE
	// --------------------------------------------------------------------------

	printf("READ MAVLINK\n");

//	read_message();

	printf("\n");


	// --------------------------------------------------------------------------
	//   STREAM COMMANDS
	// --------------------------------------------------------------------------

	printf("Start Off-Board Mode\n");
	
	if(toggle_offboard(1.0))
	{
		mavlink_local_position_ned_t pos;

		float sp_x = 0.0f;
		float sp_y = 0.0f;
		float sp_z = -0.2f;		//Take off to height of 0.2m above take-off point
		float sp_yaw = 0.0f;
		/**
		 * Pixhawk needs to see off-board commands at minimum 2Hz, otherwise it
		 * will go into fail safe mode (and probably descend)
		 */
		printf("Write Off-Board Commands\n");

		while(CMD_STREAM_FLAG)
		{
			usleep(250000);   // Stream at 4 Hz
			write_message(sp_x, sp_y, sp_z, sp_yaw);
			
			if(pos.x == sp_x || pos.y == sp_y || pos.z == sp_z)
			{
				//Setpoint reached, we can stop streaming now
				break;
			}
		}

		printf("Stop Off-Board Mode\n");
		toggle_offboard(0.0);

		printf("\n");
	}

	else
	{
		printf("Failed to switch to offboard. Aborting...\n");
	}

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
write_message(float x, float y, float z, float yaw)
{

	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------
	mavlink_set_position_target_local_ned_t sp;
	sp.time_boot_ms     = 0;
	sp.type_mask        = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
			              MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
	sp.target_system    = sysid;
	sp.target_component = autopilot_compid;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.x = x;   // move north one meter
	sp.y = y;
	sp.z = z;
	sp.yaw = yaw;

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

// --------------------------------------------------------------------------------
//    Handle CTRL-C terminate commands from the terminal
// -------------------------------------------------------------------------------

void
quit_handler(int sig)
{
	printf("Terminating at user's request\n");
	CMD_STREAM_FLAG = 0;
}
