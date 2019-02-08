/*
 * pid_control_float.cpp
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#include "../src/Header/pid_control_float.h"

void PID_Control_Float(PID* dst, double target, double input)
{
	dst->nowValue = input;
	dst->target = target;

	dst->nowError = dst->nowValue - dst->target;
	dst->errorSum += dst->nowError;
	dst->errorDiff = dst->nowError - dst->pastError;
	if(dst->errorSumLimit !=0)
	{
		if(dst->errorSum > dst->errorSumLimit)
			dst->errorSum = dst->errorSumLimit;
		else if(dst->errorSum < -dst->errorSumLimit)
			dst->errorSum = -dst->errorSumLimit;
	}
	dst->nowOutput =
			dst->kP * dst->nowError +
			dst->kI * dst->errorSum +
			dst->kD * dst->errorDiff;

	if(dst->underOfPoint == 0) return;

	dst->nowOutput /= dst -> underOfPoint;

	dst->pastError = dst->nowError;

	if(dst->nowOutput > dst->outputLimit) dst->nowOutput = dst->outputLimit;
	else if(dst->nowOutput < -dst->outputLimit) dst->nowOutput = -dst->outputLimit;
}



