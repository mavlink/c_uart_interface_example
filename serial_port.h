
#include <cstdlib>
#include <stdio.h>   /* Standard input/output definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <common/mavlink.h>

// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif


int open_port(const char* port);
bool setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
void close_port();

int start_serial(char *&uart_name, int &baudrate);

int read_serial(mavlink_message_t &message);
int write_serial(mavlink_message_t &message);
