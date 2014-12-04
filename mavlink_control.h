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
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

using std::string;
using namespace std;

#include <common/mavlink.h>

#include "autopilot_interface.h"
#include "serial_port.h"


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int main(int argc, char **argv);
int top(int argc, char **argv);

void commands(Autopilot_Interface &autopilot_interface);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);

// quit handler
Autopilot_Interface *autopilot_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

