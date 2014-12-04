#include "offboard_setup.h"
#include "serial_port.h"
#include "system_ids.h"


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
start_offboard()
{
	// ----------------------------------------------------------------------
	//   TOGGLE OFF-BOARD MODE
	// ----------------------------------------------------------------------

	// Sends the command to go off-board
	int success = toggle_offboard(1.0);

	// Check the command was written
	if ( not success )
	{
		fprintf(stderr,"Error: off-board mode not set, could not write message\n");
		throw EXIT_FAILURE;
	}
}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
stop_offboard(void)
{
	// ----------------------------------------------------------------------
	//   TOGGLE OFF-BOARD MODE
	// ----------------------------------------------------------------------

	// Sends the command to stop off-board
	int success = toggle_offboard(0.0);

	// Check the command was written
	if ( not success )
	{
		fprintf(stderr,"Error: off-board mode not set, could not write message\n");
		throw EXIT_FAILURE;
	}

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
toggle_offboard(float sw)
{
	// Prepare command for off-board mode
	mavlink_command_long_t com;
	
	/* This paramater is what sets the offboard mode
	   param1 > 0.5f = Enter offboard mode
	   param2 < 0.5f = Exit offboard mode
	*/
	com.param1           = sw;
	com.target_system    = sysid;
	com.target_component = autopilot_compid;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(sysid, compid, &message, &com);

	// Send the message
	int len = write_serial(message);

	// Check if mode is successfully set
	if(len > 0)
		return 1;
	else
		return 0;

}






