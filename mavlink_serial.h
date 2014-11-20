
// Standard includes
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


int main_read();
int main_send();
int parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);
void quit_handler(int sig);

