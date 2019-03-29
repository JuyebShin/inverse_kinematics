/*
 * myARS.h
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#ifndef MYARS_H_
#define MYARS_H_

#include <string>
#include <iostream>
#include <cstring>
#include <cstdlib>

using namespace std;
typedef struct _myARS{
	int rawData[8];
	double AX;
	double AY;
	double AZ;
	double GX;
	double GY;
	double TT;
	double RA;
	double PA;
} myARS;

void imuDataParse(char* data, myARS* imu);




#endif /* MYARS_H_ */
