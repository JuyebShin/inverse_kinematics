/*
 * Serial.cpp
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#include "../src/Header/Serial.h"
#include <iostream>

using namespace std;

SerialPort::SerialPort(char *dev_name, int baud) {
    OpenPort(dev_name,baud,0,0);
}
SerialPort::SerialPort(char *dev_name, int baud, int vtime, int vmin) {
    OpenPort(dev_name,baud,vtime,vmin);
}

void SerialPort::OpenPort(char *dev_name, int baud, int vtime, int vmin){
    struct termios newtio;

    //open serial port
    cout << "Connecting to... " << dev_name << endl;
    cout << "Baud Rate... " << baud << endl;
    fd = open(dev_name, O_RDWR | O_NOCTTY);
    if(fd < 0)
    {
        //fail open
        printf("fail open");
        return;
    } else {
    	cout << "open device: " << fd << endl;

    }

    // port configure
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR;    // no-parity
    newtio.c_oflag = 0;

    newtio.c_cflag = CS8 | CLOCAL | CREAD;  // no-rts & no-cts

    switch(baud)
    {
    case 500000 : newtio.c_cflag |= B500000; break;
    case 115200 : newtio.c_cflag |= B115200; break;
    case 57600  : newtio.c_cflag |= B57600; break;
    case 9600   : newtio.c_cflag |= B9600; break;
    default     : newtio.c_cflag |= B115200; break;
    }

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = vtime;     // timeout 0.1s
    newtio.c_cc[VMIN] = vmin;       // wait

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

}
void SerialPort::ClosePort() {
    close(fd);
}
void SerialPort::SendData(char* text, int cnt) {
    write(fd, text, cnt);
}
int SerialPort::RecvData(char* text, int cnt) {
	return read(fd,text,cnt);
}
void SerialPort::RecvLine(char* text, int cnt){
	stringArr = new char[cnt];
	char* ptr = stringArr;
	int i = 0;
	while(i<cnt) {
		char tmp = 0;
		int n = read(fd,&tmp,1);
		if(tmp != ' ' && n == 1) {
			if(tmp == '\n' || tmp == '\r'){
				break;
			}
			*ptr = tmp;
			ptr++;
			i++;
		}
	}
	text = stringArr;
}
SerialPort::~SerialPort() {
    ClosePort();
}


