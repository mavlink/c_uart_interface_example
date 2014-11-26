/** This example is public domain. */

/**
 * @file system_ids.h
 *
 * @brief System ID Numbers
 *
 * Defines the system id and component id
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock, <jaycee.lock@gmail.com>
 *
 */

#ifndef SYSTEM_IDS_H_
#define SYSTEM_IDS_H_


// ------------------------------------------------------------------------------
//   Parameters
// ------------------------------------------------------------------------------

/*int sysid            = 1;   // The vehicle's system ID (parameter: MAV_SYS_ID)
int autopilot_compid = 50;  // The autopilot component (parameter: MAV_COMP_ID)
int compid           = 355; // The offboard computer component ID */

//#define sysid 1
//#define autopilot_compid 50
//#define compid 355

const unsigned int sysid = 1;
const unsigned int autopilot_compid = 50;
const unsigned int compid = 355;


#endif // SYSTEM_IDS_H_
