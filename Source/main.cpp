// //
// //
// // #include <unistd.h>
// //
// // #include <stdio.h>
// // #include <string.h>
// // #include <unistd.h>
// // #include <fcntl.h>
// // #include <sys/ioctl.h>
// // #include <errno.h>
// // #include <paths.h>
// // #include <termios.h>
// // #include <sysexits.h>
// // #include <sys/param.h>
// // #include <sys/select.h>
// // #include <sys/time.h>
// // #include <time.h>
// //
// // #include <CoreFoundation/CoreFoundation.h>
// // #include <CoreFoundation/CFString.h>
// //
// // #include <CoreFoundation/CFBase.h>
// // #include <CoreFoundation/CFArray.h>
// // #include <CoreFoundation/CFData.h>
// // #include <CoreFoundation/CFDictionary.h>
// // #include <CoreFoundation/CFCharacterSet.h>
// // #include <CoreFoundation/CFLocale.h>
// //
// // #include <IOKit/IOKitLib.h>
// // #include <IOKit/serial/IOSerialKeys.h>
// // #include <IOKit/IOBSD.h>
// //
// // #define LOCAL_ECHO
// //
// // #ifdef LOCAL_ECHO
// // #define kOKResponseString "AT\r\r\nOK\r\n"
// // #else
// // #define kOKResponseString "\r\nOK\r\n"
// // #endif
// //
// // #define kATCommandString        "AT\r"
// // #define kMyErrReturn            -1
// //
// // enum
// // {
// //     kNumRetries = 3
// // };
// //
// // static kern_return_t MyFindModems(io_iterator_t *matchingServices)
// // {
// //     kern_return_t       kernResult;
// //     mach_port_t         masterPort;
// //     CFMutableDictionaryRef  classesToMatch;
// //
// //     kernResult = IOMasterPort(MACH_PORT_NULL, &masterPort);
// //     if (KERN_SUCCESS != kernResult)
// //     {
// //         printf("IOMasterPort returned %d\n", kernResult);
// //     goto exit;
// //     }
// //
// //     // Serial devices are instances of class IOSerialBSDClient.
// //     classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);
// //     if (classesToMatch == NULL)
// //     {
// //         printf("IOServiceMatching returned a NULL dictionary.\n");
// //     }
// //     else {
// //         CFDictionarySetValue(classesToMatch,
// //                              CFSTR(kIOSerialBSDTypeKey),
// //                              CFSTR(kIOSerialBSDModemType));
// //
// //         // Each serial device object has a property with key
// //         // kIOSerialBSDTypeKey and a value that is one of
// //         // kIOSerialBSDAllTypes, kIOSerialBSDModemType,
// //         // or kIOSerialBSDRS232Type. You can change the
// //         // matching dictionary to find other types of serial
// //         // devices by changing the last parameter in the above call
// //         // to CFDictionarySetValue.
// //     }
// //
// //     kernResult = IOServiceGetMatchingServices(masterPort, classesToMatch, matchingServices);
// //     if (KERN_SUCCESS != kernResult)
// //     {
// //         printf("IOServiceGetMatchingServices returned %d\n", kernResult);
// //     goto exit;
// //     }
// //
// // exit:
// //     return kernResult;
// // }
// //
// // static kern_return_t MyGetModemPath(io_iterator_t serialPortIterator, char *deviceFilePath, CFIndex maxPathSize)
// // {
// //     io_object_t     modemService;
// //     kern_return_t   kernResult = KERN_FAILURE;
// //     Boolean     modemFound = false;
// //
// //     // Initialize the returned path
// //     *deviceFilePath = '\0';
// //
// //     // Iterate across all modems found. In this example, we exit after
// //     // finding the first modem.
// //
// //     while ((!modemFound) && (modemService = IOIteratorNext(serialPortIterator)))
// //     {
// //         CFTypeRef   deviceFilePathAsCFString;
// //
// //     // Get the callout device's path (/dev/cu.xxxxx).
// //     // The callout device should almost always be
// //     // used. You would use the dialin device (/dev/tty.xxxxx) when
// //     // monitoring a serial port for
// //     // incoming calls, for example, a fax listener.
// //
// //     deviceFilePathAsCFString = IORegistryEntryCreateCFProperty(modemService,
// //                             CFSTR(kIOCalloutDeviceKey),
// //                             kCFAllocatorDefault,
// //                             0);
// //         if (deviceFilePathAsCFString)
// //         {
// //             Boolean result;
// //
// //         // Convert the path from a CFString to a NULL-terminated C string
// //         // for use with the POSIX open() call.
// //
// //         result = CFStringGetCString(deviceFilePathAsCFString,
// //                                         deviceFilePath,
// //                                         maxPathSize,
// //                                         kCFStringEncodingASCII);
// //             CFRelease(deviceFilePathAsCFString);
// //
// //             if (result)
// //             {
// //                 printf("BSD path: %s", deviceFilePath);
// //                 modemFound = true;
// //                 kernResult = KERN_SUCCESS;
// //             }
// //         }
// //
// //         printf("\n");
// //
// //         // Release the io_service_t now that we are done with it.
// //
// //     (void) IOObjectRelease(modemService);
// //     }
// //
// //     return kernResult;
// // }
// //
// // static int MyOpenSerialPort(const char *deviceFilePath)
// // {
// //     int         fileDescriptor = -1;
// //     int         handshake;
// //     struct termios  options;
// //
// //     // Open the serial port read/write, with no controlling terminal,
// //     // and don't wait for a connection.
// //     // The O_NONBLOCK flag also causes subsequent I/O on the device to
// //     // be non-blocking.
// //     // See open(2) ("man 2 open") for details.
// //
// //     fileDescriptor = open(deviceFilePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
// //     if (fileDescriptor == -1)
// //     {
// //         printf("Error opening serial port %s - %s(%d).\n",
// //                deviceFilePath, strerror(errno), errno);
// //         goto error;
// //     }
// //
// //     // Note that open() follows POSIX semantics: multiple open() calls to
// //     // the same file will succeed unless the TIOCEXCL ioctl is issued.
// //     // This will prevent additional opens except by root-owned processes.
// //     // See tty(4) ("man 4 tty") and ioctl(2) ("man 2 ioctl") for details.
// //
// //     if (ioctl(fileDescriptor, TIOCEXCL) == kMyErrReturn)
// //     {
// //         printf("Error setting TIOCEXCL on %s - %s(%d).\n",
// //             deviceFilePath, strerror(errno), errno);
// //         goto error;
// //     }
// //
// //     // Now that the device is open, clear the O_NONBLOCK flag so
// //     // subsequent I/O will block.
// //     // See fcntl(2) ("man 2 fcntl") for details.
// //
// //     if (fcntl(fileDescriptor, F_SETFL, 0) == kMyErrReturn)
// //     {
// //         printf("Error clearing O_NONBLOCK %s - %s(%d).\n",
// //             deviceFilePath, strerror(errno), errno);
// //         goto error;
// //     }
// //
// //     // Get the current options and save them so we can restore the
// //     // default settings later.
// //     if (tcgetattr(fileDescriptor, &gOriginalTTYAttrs) == kMyErrReturn)
// //     {
// //         printf("Error getting tty attributes %s - %s(%d).\n",
// //             deviceFilePath, strerror(errno), errno);
// //         goto error;
// //     }
// //
// //     // The serial port attributes such as timeouts and baud rate are set by
// //     // modifying the termios structure and then calling tcsetattr to
// //     // cause the changes to take effect. Note that the
// //     // changes will not take effect without the tcsetattr() call.
// //     // See tcsetattr(4) ("man 4 tcsetattr") for details.
// //
// //     options = gOriginalTTYAttrs;
// //
// //     // Print the current input and output baud rates.
// //     // See tcsetattr(4) ("man 4 tcsetattr") for details.
// //
// //     printf("Current input baud rate is %d\n", (int) cfgetispeed(&options));
// //     printf("Current output baud rate is %d\n", (int) cfgetospeed(&options));
// //
// //     // Set raw input (non-canonical) mode, with reads blocking until either
// //     // a single character has been received or a one second timeout expires.
// //     // See tcsetattr(4) ("man 4 tcsetattr") and termios(4) ("man 4 termios")
// //     // for details.
// //
// //     cfmakeraw(&options);
// //     options.c_cc[VMIN] = 1;
// //     options.c_cc[VTIME] = 10;
// //
// //     // The baud rate, word length, and handshake options can be set as follows:
// //
// //     cfsetspeed(&options, B19200);   // Set 19200 baud
// //     options.c_cflag |= (CS7        |// Use 7 bit words
// //             PARENB     |        // Enable parity (even parity if PARODD
// //                                 // not also set)
// //             CCTS_OFLOW |        // CTS flow control of output
// //             CRTS_IFLOW);        // RTS flow control of input
// //
// //     // Print the new input and output baud rates.
// //
// //     printf("Input baud rate changed to %d\n", (int) cfgetispeed(&options));
// //     printf("Output baud rate changed to %d\n", (int) cfgetospeed(&options));
// //
// //     // Cause the new options to take effect immediately.
// //     if (tcsetattr(fileDescriptor, TCSANOW, &options) == kMyErrReturn)
// //     {
// //         printf("Error setting tty attributes %s - %s(%d).\n",
// //             deviceFilePath, strerror(errno), errno);
// //         goto error;
// //     }
// //
// //     // To set the modem handshake lines, use the following ioctls.
// //     // See tty(4) ("man 4 tty") and ioctl(2) ("man 2 ioctl") for details.
// //
// //     if (ioctl(fileDescriptor, TIOCSDTR) == kMyErrReturn)
// //     // Assert Data Terminal Ready (DTR)
// //     {
// //         printf("Error asserting DTR %s - %s(%d).\n",
// //             deviceFilePath, strerror(errno), errno);
// //     }
// //
// //     if (ioctl(fileDescriptor, TIOCCDTR) == kMyErrReturn)
// //     // Clear Data Terminal Ready (DTR)
// //     {
// //         printf("Error clearing DTR %s - %s(%d).\n",
// //             deviceFilePath, strerror(errno), errno);
// //     }
// //
// //     handshake = TIOCM_DTR | TIOCM_RTS | TIOCM_CTS | TIOCM_DSR;
// //     // Set the modem lines depending on the bits set in handshake.
// //     if (ioctl(fileDescriptor, TIOCMSET, &handshake) == kMyErrReturn)
// //     {
// //         printf("Error setting handshake lines %s - %s(%d).\n",
// //             deviceFilePath, strerror(errno), errno);
// //     }
// //
// //     // To read the state of the modem lines, use the following ioctl.
// //     // See tty(4) ("man 4 tty") and ioctl(2) ("man 2 ioctl") for details.
// //
// //     if (ioctl(fileDescriptor, TIOCMGET, &handshake) == kMyErrReturn)
// //     // Store the state of the modem lines in handshake.
// //     {
// //         printf("Error getting handshake lines %s - %s(%d).\n",
// //             deviceFilePath, strerror(errno), errno);
// //     }
// //
// //     printf("Handshake lines currently set to %d\n", handshake);
// //
// //     // Success:
// //     return fileDescriptor;
// //
// //     // Failure:
// // error:
// //     if (fileDescriptor != kMyErrReturn)
// //     {
// //         close(fileDescriptor);
// //     }
// //
// //     return -1;
// // }
// //
// // static Boolean MyInitializeModem(int fileDescriptor)
// // {
// //     char    buffer[256];    // Input buffer
// //     char    *bufPtr;        // Current char in buffer
// //     ssize_t numBytes;       // Number of bytes read or written
// //     int     tries;          // Number of tries so far
// //     Boolean result = false;
// //
// //     for (tries = 1; tries <= kNumRetries; tries++)
// //     {
// //         printf("Try #%d\n", tries);
// //
// //         // Send an AT command to the modem
// //         numBytes = write(fileDescriptor, kATCommandString,
// //                          strlen(kATCommandString));
// //
// //     if (numBytes == kMyErrReturn)
// //         {
// //             printf("Error writing to modem - %s(%d).\n", strerror(errno),
// //                         errno);
// //             continue;
// //         }
// //     else {
// //         printf("Wrote %d bytes \"%s\"\n", numBytes,
// //                         MyLogString(kATCommandString));
// //     }
// //
// //     if (numBytes < strlen(kATCommandString))
// //     {
// //             continue;
// //     }
// //
// //         printf("Looking for \"%s\"\n", MyLogString(kOKResponseString));
// //
// //     // Read characters into our buffer until we get a CR or LF.
// //         bufPtr = buffer;
// //         do
// //         {
// //             numBytes = read(fileDescriptor, bufPtr, &buffer[sizeof(buffer)]
// //                         - bufPtr - 1);
// //             if (numBytes == kMyErrReturn)
// //             {
// //                 printf("Error reading from modem - %s(%d).\n", strerror(errno),
// //                         errno);
// //             }
// //             else if (numBytes > 0)
// //             {
// //                 bufPtr += numBytes;
// //                 if (*(bufPtr - 1) == '\n' || *(bufPtr - 1) == '\r')
// //                 {
// //                     break;
// //                 }
// //             }
// //             else {
// //                 printf("Nothing read.\n");
// //             }
// //         } while (numBytes > 0);
// //
// //         // NULL terminate the string and see if we got a response of OK.
// //         *bufPtr = '\0';
// //
// //         printf("Read \"%s\"\n", MyLogString(buffer));
// //
// //         if (strncmp(buffer, kOKResponseString, strlen(kOKResponseString)) == 0)
// //         {
// //             result = true;
// //             break;
// //         }
// //     }
// //
// //     return result;
// // }
// //
// // static char *MyLogString(char *str)
// // {
// //     static char     buf[2048];
// //     char            *ptr = buf;
// //     int             i;
// //
// //     *ptr = '\0';
// //
// //     while (*str)
// //     {
// //         if (isprint(*str))
// //         {
// //             *ptr++ = *str++;
// //         }
// //         else {
// //             switch(*str)
// //             {
// //             case ' ':
// //                 *ptr++ = *str;
// //                 break;
// //
// //             case 27:
// //                 *ptr++ = '\\';
// //                 *ptr++ = 'e';
// //                 break;
// //
// //             case '\t':
// //                 *ptr++ = '\\';
// //                 *ptr++ = 't';
// //                 break;
// //
// //             case '\n':
// //                 *ptr++ = '\\';
// //                 *ptr++ = 'n';
// //                 break;
// //
// //             case '\r':
// //                 *ptr++ = '\\';
// //                 *ptr++ = 'r';
// //                 break;
// //
// //             default:
// //                 i = *str;
// //                 (void)sprintf(ptr, "\\%03o", i);
// //                 ptr += 4;
// //                 break;
// //             }
// //
// //             str++;
// //         }
// //         *ptr = '\0';
// //     }
// //     return buf;
// // }
// //
// // void MyCloseSerialPort(int fileDescriptor)
// // {
// //     // Block until all written output has been sent from the device.
// //     // Note that this call is simply passed on to the serial device driver.
// //     // See tcsendbreak(3) ("man 3 tcsendbreak") for details.
// //     if (tcdrain(fileDescriptor) == kMyErrReturn)
// //     {
// //         printf("Error waiting for drain - %s(%d).\n",
// //             strerror(errno), errno);
// //     }
// //
// //     // It is good practice to reset a serial port back to the state in
// //     // which you found it. This is why we saved the original termios struct
// //     // The constant TCSANOW (defined in termios.h) indicates that
// //     // the change should take effect immediately.
// //     if (tcsetattr(fileDescriptor, TCSANOW, &gOriginalTTYAttrs) ==
// //                     kMyErrReturn)
// //     {
// //         printf("Error resetting tty attributes - %s(%d).\n",
// //                     strerror(errno), errno);
// //     }
// //
// //     close(fileDescriptor);
// // }
// //
// //
// // int main()
// // {
// //     // int data[] = {10,5,13};  //Random data we want to send
// //     // //FILE *file;
// //     // //file = fopen("/dev/cu.usbmodem1411","w");  //Opening device file
// //     // int i = 0;
// //     // ofstream f("/dev/cu.usbmodem1411");
// //     // for(i = 0 ; i < 10000 ; i++)
// //     // {
// //     //     //fprintf(file,"%d",data[i]); //Writing to the file
// //     //     //fprintf(file,"%c",','); //To separate digits
// //     //     f << "bite";
// //     //     sleep(1);
// //     // }
// //     // //fclose(file);
// //     int         fileDescriptor;
// //     kern_return_t   kernResult;
// //
// //     io_iterator_t   serialPortIterator;
// //     char        deviceFilePath[MAXPATHLEN];
// //
// //     kernResult = MyFindModems(&serialPortIterator);
// //
// //     kernResult = MyGetModemPath(serialPortIterator, deviceFilePath,
// //                      sizeof(deviceFilePath));
// //
// //     IOObjectRelease(serialPortIterator);    // Release the iterator.
// //
// //     // Open the modem port, initialize the modem, then close it.
// //     if (!deviceFilePath[0])
// //     {
// //         printf("No modem port found.\n");
// //         return EX_UNAVAILABLE;
// //     }
// //
// //     fileDescriptor = MyOpenSerialPort(deviceFilePath);
// //     if (fileDescriptor == kMyErrReturn)
// //     {
// //         return EX_IOERR;
// //     }
// //
// //     if (MyInitializeModem(fileDescriptor))
// //     {
// //         printf("Modem initialized successfully.\n");
// //     }
// //     else {
// //         printf("Could not initialize modem.\n");
// //     }
// //
// //     MyCloseSerialPort(fileDescriptor);
// //     printf("Modem port closed.\n");
// //
// //     return EX_OK;
// // }
// #include <stdio.h>   /* Standard input/output definitions */
// #include <string.h>  /* String function definitions */
// #include <unistd.h>  /* UNIX standard function definitions */
// #include <fcntl.h>   /* File control definitions */
// #include <errno.h>   /* Error number definitions */
// #include <termios.h> /* POSIX terminal control definitions */
//
// /*
//  * 'open_port()' - Open serial port 1.
//  *
//  * Returns the file descriptor on success or -1 on error.
//  */
// int open_port(void)
// {
//   int fd; /* File descriptor for the port */
//
//   fd = open("/dev/cu.usbmodem1411", O_RDWR | O_NOCTTY | O_NDELAY);
//   if (fd == -1)
//     {
//       /*
//        * Could not open the port.
//        */
//
//       perror("open_port: Unable to open /dev/cu.usbmodem1411 - ");
//     }
//   else
//     fcntl(fd, F_SETFL, FNDELAY);
//
//   printf ( "In Open port fd = %i\n", fd);
//
//   return (fd);
// }
//
// int main()
// {
//   int fd;  // File descriptor
//   int n;
//
//   fd = open_port();
//
//   // Read the configureation of the port
//
//   struct termios options;
//   tcgetattr( fd, &options );
//
//   /* SEt Baud Rate */
//
//   cfsetispeed( &options, B9600 );
//   cfsetospeed( &options, B9600 );
//
//   //I don't know what this is exactly
//
//   options.c_cflag |= ( CLOCAL | CREAD );
//
//   // Set the Charactor size
//
//   options.c_cflag &= ~CSIZE; /* Mask the character size bits */
//   options.c_cflag |= CS8;    /* Select 8 data bits */
//
//   // Set parity - No Parity (8N1)
//
//   options.c_cflag &= ~PARENB;
//   options.c_cflag &= ~CSTOPB;
//   options.c_cflag &= ~CSIZE;
//   options.c_cflag |= CS8;
//
//   // Disable Hardware flowcontrol
//
//   //  options.c_cflag &= ~CNEW_RTSCTS;  -- not supported
//
//   // Enable Raw Input
//
//   options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//
//   // Disable Software Flow control
//
//   options.c_iflag &= ~(IXON | IXOFF | IXANY);
//
//   // Chose raw (not processed) output
//
//   options.c_oflag &= ~OPOST;
//
//   if ( tcsetattr( fd, TCSANOW, &options ) == -1 )
//     printf ("Error with tcsetattr = %s\n", strerror ( errno ) );
//   else
//     printf ( "%s\n", "tcsetattr succeed" );
//
//   fcntl(fd, F_SETFL, FNDELAY);
//
//
//   // Write some stuff !!!
//
//   n = write(fd, "ATZ\r", 4);
//   if (n < 0)
//     fputs("write() of 4 bytes failed!\n", stderr);
//   else
//     printf ("Write succeed n = %i\n", n );
//
//
//     char buff;
//   n = read( fd, &buff, 1 );
//
//   if ( n == -1 )
//       printf ( "Error = %s\n", strerror( errno ) );
//
//   printf ( "Number of bytes to be read = %i\n", n );
//   printf ( "Buff = %c\n", buff );
//
//
//   close( fd );
//
// }
//
//
//
//
// #include <stdio.h>
//   #include <unistd.h>
//   #include <fcntl.h>
//   #include <termios.h>
//
// int main()
// {
//            //  int fd; /* port file descriptor */
//            //  FILE *file;
//            //  speed_t baud = B9600;
//            //  file = fopen("/dev/cu.usbmodem1411","w");
//            //  fd = fileno(file); /* connect to port */
//            //  usleep(3500000); // Arduino's reboot
//            //
//            // /* set the other settings (in this case, 38400 8N1) */
//            //  struct termios settings;
//            //  tcgetattr(fd, &settings);
//            //
//            //  cfsetospeed(&settings, baud); /* baud rate */
//            //  settings.c_cflag &= ~PARENB; /* no parity */
//            //  settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
//            //  settings.c_cflag &= ~CSIZE;
//            //  settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
//            //  settings.c_lflag = ICANON; /* canonical mode */
//            //  settings.c_oflag &= ~OPOST; /* raw output */
//            //
//            //  tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
//            //  tcflush(fd, TCOFLUSH);
//            //  while(1)
//            //  {
//            //      fprintf(file, "C");
//            //      sleep(1);
//            //  }
//            //  close(fd);
//            //  return 0;
//
// }

//#include "SerialPort.h"

//int main()
//{
//    Port arduino("/dev/cu.usbmodem1411");
//    arduino.write("c");
//    return 0;
//}
//
// #include <QSerialPort>

// #include "Serial.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */

// int
// open_port(void)
// {
//   int fd; /* File descriptor for the port */
//
//
//   fd = open("/dev/cu.usbmodem1411", O_RDWR | O_NOCTTY | O_NDELAY);
//   if (fd == -1)
//   {
//    /*
//     * Could not open the port.
//     */
//
//     perror("open_port: Unable to open /dev/ttyf1 - ");
//   }
//   else
//     fcntl(fd, F_SETFL, 0);
//
//   return (fd);
// }
//
// int main()
// {
//     int fd = open_port();
//     sleep(5);
//     int k = 0;
//     while(k< 10)
//     {
//         int n = write(fd, "a", 1);
//         if (n < 0)
//           fputs("write() of 4 bytes failed!\n", stderr);
//         usleep(100000);
//         k++;
//     }
//     return 0;
//     close(fd);
// }

int bpmToUS(float bpm)
{
    return int(1000000*60/bpm);
}

#include "SerialPort.h"
#include "animation.h"

int main()
{
    SerialPort arduino("/dev/cu.usbmodem1411");
    for(int i=0; i<50; i++)
    {
        flash(arduino);
        usleep(bpmToUS(500));
    }
    return 0;
}
