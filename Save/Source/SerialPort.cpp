#include "SerialPort.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <string>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

using namespace std;

SerialPort::SerialPort(char* givenPath)
{
    path = givenPath;

    fd = open(path, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
      perror("open_port: Unable to open port - ");
    else
      fcntl(fd, F_SETFL, 0);

    //Giving time to arduino to initialize
    sleep(1);
}

SerialPort::~SerialPort()
{
    close(fd);
}

void SerialPort::serialPrint(char* c)
{
    int n = write(fd, c, 1);
    if (n < 0)
      fputs("writing failed!\n", stderr);
}
