/** This example is public domain. */

/**
 * @file system_ids.h
 *
 * @brief System ID Numbers
 *
 * Defines the system id and component id
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *
 */

#ifndef SYSTEM_IDS_H_
#define SYSTEM_IDS_H_

// ------------------------------------------------------------------------------
//   Parameters
// ------------------------------------------------------------------------------

// system parameters, default 0 to discover on first connect
int sysid            = 0;   // The vehicle's system ID (parameter: MAV_SYS_ID)
int autopilot_compid = 0;   // The autopilot component (parameter: MAV_COMP_ID)

// The offboard computer component ID
int compid           = 355;


#endif // SYSTEM_IDS_H_


