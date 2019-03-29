/*
 * Moving_point.h
 *
 *  Created on: Jan 1, 2013
 *      Author: odroid
 */

#ifndef MOVING_POINT_H_
#define MOVING_POINT_H_

#include <iostream>
#include <cmath>

class Moving_point{
private:
	double X;
	double Y;
	double Z;


public:
	double XF;
	double YF;
	double ZF;

	Moving_point(double X2,double Y2,double Z2){
		X = X2;	Y = Y2; Z = Z2; XF = 0; YF = 0; ZF = 0;
	}
	void Setting_point(double X1,double Y1,double Z1);
	void Rotation_point(double thetaX,double thetaY,double thetaZ);

};



#endif /* MOVING_POINT_H_ */
