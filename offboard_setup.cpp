#include "offboard_setup.h"
#include "serial_port.h"
#include "system_ids.h"

// ------------------------------------------------------------------------------
//   Parameters
// ------------------------------------------------------------------------------

bool offboard_status = false;

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
toggle_offboard(void)
{
	// Prepare command for off-board mode
	mavlink_command_long_t com;
	com.param1           = 1.0f;   //A number > 0.5f is required here
	com.target_system    = sysid;
	com.target_component = autopilot_compid;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(sysid, compid, &message, &com);

	// Send the message
	int len = write_serial(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
start_offboard()
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
		int len = write_serial(message);
		if ( not len )
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			throw EXIT_FAILURE;
		}


		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard();

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
stop_offboard(void)
{
	// Should only send this command once
	if ( offboard_status )
	{

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard();

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



