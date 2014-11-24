/** This example is public domain. */

/**
 * @file mavlink_control.h
 *
 * @brief The serial interface definition
 *
 * This process connects an external MAVLink UART device to send an receive data
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

#include "serial_port.h"

#include <common/mavlink.h>


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int read_message();
int write_message();
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);

