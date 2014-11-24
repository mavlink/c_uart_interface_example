/** This example is public domain. */

/**
 * @file serial_port.h
 *
 * @brief Serial interface definition
 *
 * Functions for opening, closing, reading and writing via serial ports
  *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <cstdlib>
#include <stdio.h>   /* Standard input/output definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <common/mavlink.h>


// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int read_serial(mavlink_message_t &message);
int write_serial(mavlink_message_t &message);

void open_serial(char *&uart_name, int &baudrate);
void close_serial();

int _open_port(const char* port);
bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);


