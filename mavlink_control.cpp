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
#include "serial_port.cpp"
#include "system_ids.h"
#include "offboard_setup.h"
#include "pos_target_bitmasks.h"


// ------------------------------------------------------------------------------
//   Parameters
// ------------------------------------------------------------------------------

Serial_Port serial_port;

Vehicle_Data vehicle_data;

bool time_to_exit = false;
bool write_thread_running = false;

uint64_t write_count = 0;

mavlink_set_position_target_local_ned_t position_target;
mavlink_set_position_target_local_ned_t position_initial;

pthread_t read_tid;
pthread_t write_tid;


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------
	startup( argc , argv );

	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------
	commands();

	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------
	shutdown();

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands()
{

	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	printf("START OFFBOARD MODE\n");
	start_offboard(serial_port);

	// now pixhawk is accepting setpoint commands
	printf("\n");


	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------

	// prepare command
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.x   = position_initial.x;
	sp.y   = position_initial.y;
	sp.z   = position_initial.z;
	sp.yaw = position_initial.yaw;

	// set command
	position_target = sp;

	// now pixhawk will try to move
	sleep(8);


	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------
	printf("\n");
	printf("STOP OFFBOARD MODE\n");

	stop_offboard(serial_port);

	sleep(2);

	printf("STOPPED\n");

	// now pixhawk isn't listening to setpoint commands
	printf("\n");

}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------

void startup(int argc, char **argv)
{
	// Default input arguments
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;
	int result;

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// will throw an int if fails
	parse_commandline(argc, argv, uart_name, baudrate);


	// --------------------------------------------------------------------------
	//   START SERIAL PORT
	// --------------------------------------------------------------------------
	printf("OPEN PORT\n");

	// will throw an int if fails
	serial_port.open_serial(uart_name, baudrate);

	// now we can read from and write to pixhawk
	printf("\n");


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------
	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &read_thread, NULL );
	if ( result ) throw result;

	// now we're reading pixhawk's messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR HEARTBEAT
	// --------------------------------------------------------------------------
	printf("CHECK FOR HEARTBEAT\n");

	while ( not vehicle_data.time_stamps.heartbeat )
		usleep(500000);

	printf("FOUND\n");

	// now we know pixhawk is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to.  If there is more than one
	// vehicle than probably can't expect to discover id's like this

	// System ID
	if ( not sysid )
	{
		sysid = vehicle_data.sysid;
		printf("GOT VEHICLE SYSID:    %i\n", sysid );
	}

	// Component ID
	if ( not autopilot_compid )
	{
		autopilot_compid = vehicle_data.compid;
		printf("GOT AUTOPILOT COMPID: %i\n", autopilot_compid);
		printf("\n");
	}

	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
	while ( not ( vehicle_data.time_stamps.local_position_ned &&
			  	  vehicle_data.time_stamps.attitude            )  )
	{
		usleep(500000);
	}

	// copy initial position ned
	Vehicle_Data local_data = vehicle_data;
	position_initial.x            = local_data.local_position_ned.x;
	position_initial.y            = local_data.local_position_ned.y;
	position_initial.z            = local_data.local_position_ned.z;
	position_initial.vx           = local_data.local_position_ned.vx;
	position_initial.vy           = local_data.local_position_ned.vy;
	position_initial.vz           = local_data.local_position_ned.vz;
	position_initial.yaw          = local_data.attitude.yaw;
	position_initial.yaw_rate     = local_data.attitude.yawspeed;

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");
	printf("\n");

	result = pthread_create( &write_tid, NULL, &write_thread, NULL );
	if ( result ) throw result;

	while ( not write_thread_running )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------

void shutdown()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");
	printf("\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// --------------------------------------------------------------------------
	//   CLOSE PORT
	// --------------------------------------------------------------------------
	printf("CLOSE PORT\n");

	serial_port.close_serial();

	// now the serial port is closed
	printf("\n");
}


// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------

void*
read_thread(void *arg)
{
	while ( not time_to_exit )
	{
		read_message();
		usleep(100000); // Read batches at 10Hz
	}
	return NULL;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------

int
read_message()
{
	bool success;           // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( not received_all )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port.read_serial(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{
			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(vehicle_data.heartbeat));
					vehicle_data.sysid  = message.sysid;
					vehicle_data.compid = message.compid;
					vehicle_data.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = vehicle_data.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(vehicle_data.sys_status));
					vehicle_data.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = vehicle_data.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(vehicle_data.battery_status));
					vehicle_data.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = vehicle_data.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(vehicle_data.radio_status));
					vehicle_data.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = vehicle_data.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(vehicle_data.local_position_ned));
					vehicle_data.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = vehicle_data.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(vehicle_data.global_position_int));
					vehicle_data.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = vehicle_data.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(vehicle_data.position_target_local_ned));
					vehicle_data.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = vehicle_data.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(vehicle_data.position_target_global_int));
					vehicle_data.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = vehicle_data.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(vehicle_data.highres_imu));
					vehicle_data.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = vehicle_data.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(vehicle_data.attitude));
					vehicle_data.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = vehicle_data.time_stamps.attitude;
					break;
				}

			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
				this_timestamps.sys_status                 &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
				this_timestamps.position_target_global_int &&
				this_timestamps.highres_imu                &&
				this_timestamps.attitude                   ;

		// give the write thread time to use the port
		if ( write_thread_running )
			usleep(100); // look for components of batches at 10kHz

	} // end: while not received all

	/*
	// Do something with the messages

	mavlink_heartbeat_t hb = vehicle_data.heartbeat;
	printf("Got message HEARTBEAT (spec: https://pixhawk.ethz.ch/mavlink/#HEARTBEAT)\n");
	printf("    type:          %i\n", hb.type);
	printf("    sysid:         %i\n", vehicle_data.sysid);
	printf("    compid:        %i\n", vehicle_data.compid);
	printf("    autopilot:     %i\n", hb.autopilot);
	printf("    base_mode:     %i\n", hb.base_mode);
	printf("    custom_mode:   %i\n", hb.custom_mode);
	printf("    system_status: %i\n", hb.system_status);
	printf("    timestamp:     %lu\n", vehicle_data.time_stamps.heartbeat);

	mavlink_sys_status_t st = vehicle_data.sys_status;
	printf("Got message SYS_STATUS (spec: https://pixhawk.ethz.ch/mavlink/#SYS_STATUS)\n");
	printf("    load:              %i\n", st.load);
	printf("    voltage_battery:   %i\n", st.voltage_battery);
	printf("    current_battery:   %i\n", st.current_battery);
	printf("    battery_remaining: %i\n", st.battery_remaining);
	printf("    timestamp:         %lu\n", vehicle_data.time_stamps.sys_status);

	mavlink_highres_imu_t imu = vehicle_data.highres_imu;
	printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
	printf("    time:        %lu \n", imu.time_usec);
	printf("    acc  (NED):  %f %f %f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
	printf("    gyro (NED):  %f %f %f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("    mag  (NED):  %f %f %f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
	printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
	printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
	printf("    temperature: %f C \n"       , imu.temperature );
	printf("    timestamp:   %lu \n"        , vehicle_data.time_stamps.sys_status);


	mavlink_local_position_ned_t pos = vehicle_data.local_position_ned;
	printf("Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

	printf("\n");

	 */

	mavlink_local_position_ned_t pos = vehicle_data.local_position_ned;
	printf("\n");
	printf("%lu POSITION_CURRENT = [ %f , %f , %f ] \n", write_count, pos.x, pos.y, pos.z);
	printf("\n");

	return 0;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------

void*
write_thread(void *arg)
{

	// prepare position target
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.x   = position_initial.x;
	sp.y   = position_initial.y;
	sp.z   = position_initial.z;
	sp.yaw = position_initial.yaw;

	// set position target
	position_target = sp;

	// write a message and signal start
	write_message();
	write_thread_running = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( not time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		write_message();
	}

	// signal end
	write_thread_running = false;

	return NULL;
}


// ------------------------------------------------------------------------------
//   Write Messages
// ------------------------------------------------------------------------------

int
write_message()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp = position_target;

	// double check some system parameters
	sp.time_boot_ms     = 0;
	sp.target_system    = sysid;
	sp.target_component = autopilot_compid;

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(sysid, compid, &message, &sp);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = serial_port.write_serial(message);

	// check the write
	if ( not len > 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	else
		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// book keep
	write_count++;

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


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------

int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		throw error;
	}

}


