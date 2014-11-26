/** This example is public domain. */

/**
 * @file offboard_setup.h
 *
 * @brief setup functions for off-board
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock, <jaycee.lock@gmail.com>
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

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// XXX should go in mavlink

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
 *
 */

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0001111111110000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b1110001111110000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b1111110001110000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b1111110001110000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b1111111111010000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b1111111111100000

// ------------------------------------------------------------------------------
//   Function prototypes
// ------------------------------------------------------------------------------
int toggle_offboard(void);
void start_offboard(void);
void stop_offboard(void);

