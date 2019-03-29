/*
 * Play_Kinematics.h
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#ifndef PLAY_KINEMATICS_H_
#define PLAY_KINEMATICS_H_


#define m_to_m 135//113

//#define DEFAULT_X           0       //-6
//#define DEFAULT_Y           -12     //0
//#define DEFAULT_Z           100      //105
//#define DEFAULT_YAW         -1       //-0.5
#define DEFAULT_X           -4      //-6
#define DEFAULT_Y           0      //0
#define DEFAULT_Z           100      //105
#define DEFAULT_YAW         -4       //-0.5

#define START_CNT   10
#define START_X     0

typedef struct info_kinematics{
	double X_length;
	double Y_length;
	double Z_length;
	double PA;
	double RA;
	double R_yaw;
	double L_yaw;
	double flag;
	int Step_counter;
	int KM_flag;
	int Using_Motor_Id[25];
	int Side_flag;
	int KM_flag2;
}Info_kinematics;


double angleToZLength(double l_knee);
double Only_Plus(double thetaz);
double Only_Plus2(double thetaz);

double cosKB(double theta,float rate);

int angleToPosition(double angle);
double positionToAngle(double Position);
double graph_calculation(double x1, double x2, double y1, double y2, double x3);

void init_save();
//void kinematics_calculation(double p_x, double p_y, double p_z_f, double leg_p, double leg_r, double leg_y_1, double leg_y_2, double motor_1415,double motor_2021,double front_value, int aaa);


void kinematics_sidewalk_left(Info_kinematics temp,double leg_p, double leg_r, int aaa);

void kinematics_calculation1(Info_kinematics temp,double leg_p, double leg_r, int aaa );
//void init_ready();


#endif /* PLAY_KINEMATICS_H_ */
