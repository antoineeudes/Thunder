#pragma once

#include <stdio.h>   /* Standard input/output definitions */
#include <string>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

using namespace std;

class SerialPort
{
   private:
       char* path;
       int fd;

   public:
       SerialPort(char* givenPath);
       ~SerialPort();

       void serialPrint(char* c);

};
