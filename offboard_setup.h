/** This example is public domain. */

/**
 * @file offboard_setup.h
 *
 * @brief setup functions for off-board
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>

using std::string;
using namespace std;

#include <common/mavlink.h>

#include "serial_port.h"
#include "system_ids.h"
#include "serial_port.h"


// ------------------------------------------------------------------------------
//   Parameters
// ------------------------------------------------------------------------------

bool offboard_status = false;


// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// TODO - should go in mavlink

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0001111111110000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b1110001111110000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b1111110001110000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b1111110001110000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b1111111111010000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b1111111111100000


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
toggle_offboard(SerialPort &serial_port)
{
	// Prepare command for off-board mode
	mavlink_command_long_t com;
	com.target_system    = sysid;             // TODO - read this from autopilot
	com.target_component = autopilot_compid;  // TODO - read this from autopilot
	com.param1           = 1.0f;   //A number > 0.5f is required here
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(sysid, compid, &message, &com);

	// Send the message
	int len = serial_port.write_serial(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
start_offboard(SerialPort &serial_port)
{
	// Should only send this command once
	if ( not offboard_status )
	{

		// ----------------------------------------------------------------------
		//   SEND INITIAL OFFBOARD COMMAND
		// ----------------------------------------------------------------------

		// The autopilot needs an initial command before going into off-board mode
		// This sends an an initial command to hold the current position and yaw angle

		// Prepare message
		mavlink_set_position_target_local_ned_t sp;
		sp.time_boot_ms     = 0;
		sp.type_mask        = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
				              MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
		sp.target_system    = sysid;
		sp.target_component = autopilot_compid;
		sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
		sp.x = 0.0f;
		sp.y = 0.0f;
		sp.z = 0.0f;
		sp.yaw = 0.0f;

		// Encode
		mavlink_message_t message;
		mavlink_msg_set_position_target_local_ned_encode(sysid, compid, &message, &sp);

		// Write
		int len = serial_port.write_serial(message);
		if ( not len )
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			throw EXIT_FAILURE;
		}


		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard(serial_port);

		// Check the command was written
		if ( success )
			offboard_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			throw EXIT_FAILURE;
		}


	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
stop_offboard(SerialPort &serial_port)
{
	// Should only send this command once
	if ( offboard_status )
	{

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard(serial_port);

		// Check the command was written
		if ( success )
			offboard_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			throw 1;
		}

	} // end: if offboard_status
}



