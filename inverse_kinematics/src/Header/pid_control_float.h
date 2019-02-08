/*
 * pid_control_float.h
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#ifndef PID_CONTROL_FLOAT_H_
#define PID_CONTROL_FLOAT_H_

typedef struct _PID{
	double nowValue;		//!<
	double pastValue;

	long nowError;
	long pastError;
	double target;

	long errorSum;
	long errorSumLimit;
	long errorDiff;

	long nowOutput;
	long pastOutput;
	long outputLimit;

	long underOfPoint;

	float kP;
	float kI;
	float kD;
}PID;


void PID_Control_Float(PID* dst, double target, double input);



#endif /* PID_CONTROL_FLOAT_H_ */
