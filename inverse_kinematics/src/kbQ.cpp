/*
 * kbQ.cpp
 *
 *  Created on: Jan 1, 2013
 *      Author: Yi gi bag
 */
#include "kbQ.h"


kbQ::kbQ(int size){
	Q_size = size;
	Q = new int[size];
	Start = 0;
	End = 0;
	Start_flag = true;
}

void kbQ::Input_value(int value){
	if(Start_flag){
		Q[End] = value;
		End ++;
	}
	else{
		if(Start == End)
			return ;
		else{
			Q[End] = value;
			End++;
			if(End == Q_size)
				End = 0;
		}
	}
}

void kbQ::Get_value(){
	if(Start_flag){
		Q[Start] = 0;
		Start ++;
	}
	else{
		if(Start == End)
			return ;
		else{
			Q[Start] = 0;
			Start++;
			if(Start == Q_size)
				Start = 0;
		}
	}
}



