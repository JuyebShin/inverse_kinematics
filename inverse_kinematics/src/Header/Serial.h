/*
 * Serial.h
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#include <termios.h>

class SerialPort{
public:
    SerialPort();
    SerialPort(char *dev_name, int baud);
    SerialPort(char *dev_name, int baud, int vtime, int vmin);

    void OpenPort(char *dev_name, int baud, int vtime, int vmin);
    void ClosePort();
    void SendData(char* text, int cnt);
    int RecvData(char* text, int cnt);
    void RecvLine(char* text, int cnt);
    ~SerialPort();

private:
    char* stringArr;
    int fd;
};




#endif /* SERIAL_H_ */
