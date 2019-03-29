/*
 * Moving_point.cpp
 *
 *  Created on: Jan 1, 2013
 *      Author: odroid
 */

#include "../src/Header/Moving_point.h"

void Moving_point::Setting_point(double X1,double Y1,double Z1){
	X = X1; Y = Y1; Z = Z1;
}

void Moving_point::Rotation_point(double thetaX,double thetaY,double thetaZ){
	thetaX = thetaX * M_PI/180;
	thetaY = thetaY * M_PI/180;
	thetaZ = thetaZ * M_PI/180;

	XF = cos(thetaY)*cos(thetaZ)*X + cos(thetaY)*sin(thetaZ)*Y * sin(thetaY)*Z;
	YF = -cos(thetaX)*sin(thetaZ)*X + cos(thetaX)*cos(thetaZ)*Y + sin(thetaX)*sin(thetaY)*cos(thetaZ)*X + sin(thetaX)*sin(thetaY)*sin(thetaZ)*Y + sin(thetaX)*cos(thetaY)*Z;
	ZF = sin(thetaX)*sin(thetaZ)*X - sin(thetaX)*cos(thetaZ)*Y + cos(thetaX)*sin(thetaY)*cos(thetaZ)*X + cos(thetaX)*sin(thetaY)*sin(thetaZ)*Y + cos(thetaX)*cos(thetaY)*Z;
}
