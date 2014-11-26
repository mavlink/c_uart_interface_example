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
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
toggle_offboard(Serial_Port &serial_port)
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
start_offboard(Serial_Port &serial_port)
{
	// Should only send this command once
	if ( not offboard_status )
	{
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
//			throw EXIT_FAILURE;
		}


	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
stop_offboard(Serial_Port &serial_port)
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
//			throw EXIT_FAILURE;
		}

	} // end: if offboard_status
}



