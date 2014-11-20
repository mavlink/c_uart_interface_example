

#include "mavlink_serial.h"
#include "system_defaults.h"



int main(int argc, char **argv)
{

	/* default values for arguments */
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;

	// PARSE THE COMMANDS
	if ( parse_commandline(argc, argv, uart_name, baudrate) )
		return 1;

	// START SERIAL PORT
	cout << "OPEN PORT" << endl;
	if ( start_serial(uart_name, baudrate) )
		return 1;

	// READ ONE MESSAGE
	cout << "READ MAVLINK" << endl;
	main_read();

	// SEND ONE MESSAGE
	cout << "SEND MAVLINK" << endl;
	main_send();

	// CLOSE PORT
	cout << "CLOSE PORT" << endl;
	close_port();

	return 0;
}



int main_read()
{
	bool success;
	bool received = false;  // receive only one message

	// Blocking wait for new data
	while (!received)
	{

		// READ MESSAGE
		mavlink_message_t message;
		success = read_serial(message);


		// DECODE MESSAGE
		if( success )
		{
			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					mavlink_highres_imu_t imu;
					mavlink_msg_highres_imu_decode(&message, &imu);

					printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
					printf("\t time: %lu\n", imu.time_usec);
					printf("\t acc  (NED):\t% f\t% f\t% f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
					printf("\t gyro (NED):\t% f\t% f\t% f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
					printf("\t mag  (NED):\t% f\t% f\t% f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
					printf("\t baro: \t %f (mBar)\n", imu.abs_pressure);
					printf("\t altitude: \t %f (m)\n", imu.pressure_alt);
					printf("\t temperature: \t %f C\n", imu.temperature);
					printf("\n");

					// Receive only one message
					received = true;
				}
				break;
			}
		}

	}

	return 0;
}



int main_send() {

	// PACK PAYLOAD
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

	// ENCODE
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(sysid, compid, &message, &sp);

	// WRITE
	int len = write_serial(message);

	printf("Sent buffer of length %i\n",len);

	return 0;
}



int parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	const char *commandline_usage = "\tusage: mavlink_serial -d <devicename> -b <baudrate>";

	/* read program arguments */
	int i;

	for (i = 1; i < argc; i++) { /* argv[0] is "mavlink" */
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			return 1;
		}

		/* UART device ID */
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				return 1;
			}
		}

		/* baud rate */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				return 1;
			}
		}

	}

	return 0;
}




