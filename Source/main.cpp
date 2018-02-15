
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "SerialPort.h"
#include "animation.h"
#include "Tools.h"

#include <thread>
#include <iostream>

bool triggerFlash = false;
bool over = false;

void severalFlash()
{
    SerialPort arduino("/dev/cu.usbmodem1411");
//    for(int i=0; i<10; i++)
//    {
//        flash(arduino);
//        usleep(bpmToUS(80));
//    }
    while(!over)
    {
        if(triggerFlash)
        {
            flash(arduino);
            triggerFlash = false;
        }
    }
}

int main()
{
    std::thread t1(severalFlash);
    for(int j = 0; j<100; j++)
    {
        cout << j << endl;
        triggerFlash = true;
        usleep(bpmToUS(77));
    }
    over = true;
    t1.join();
    return 0;
}
