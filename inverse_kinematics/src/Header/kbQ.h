/*
 * kbQ.h
 *
 *  Created on: Jan 1, 2013
 *      Author: odroid
 */

#include <iostream>
#ifndef KBQ_H_
#define KBQ_H_

class kbQ{
private :
	int Q_size;
	int *Q;
	int Start;
	int End;
	bool Start_flag;

public :
	kbQ(int size);
	void Input_value(int value);
	void Get_value();
	int Output_one_value(int number);
	int Output_average();

};



#endif /* KBQ_H_ */
