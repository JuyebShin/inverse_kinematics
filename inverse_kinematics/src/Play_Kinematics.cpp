/*
 * Play_Kinematics.cpp
 *
 *  Created on: Aug 12, 2015
 *      Author: Pae Kibaek
 */

#include <iostream>
#include <cmath>
#include "../src/Header/Moving_point.h"
#include "../src/Header/Play_Kinematics.h"
#include <fstream>
#include <string>
#include "ros/ros.h"
#include "../msg/dynamixel_msg.h"

extern bool isRunningMode;
extern int runningThetaCnt;
extern int defaultThetaCnt;

extern int Xmoved;

extern unsigned int g_DXL_ID_position[100];
extern unsigned int g_DXL_ID_INIT_position[100];
extern double g_KneeAngle;
extern int runningStartCnt;
extern int defaultStartCnt;
extern int startCnt;
extern int theta_cnt;
extern double Y_DEFAULT;
int Step = 0;

extern int Start_flag;
bool Stop = false;
bool Leg = false;

int Motion_Data[400];

extern double g_theta;
extern int theta_count;
int X_theta;
//=================================
extern int df_yr;
extern int df_yl;

extern int df_zr;
extern int df_zl;

extern int ankle_18;
extern int ankle_19;
extern int ankle_20;
extern int ankle_21;

extern int ankle_14;
extern int ankle_15;

extern int ankle_12;
extern int ankle_13;

extern int swing;
//=================================

double p_waist = 0;

double graph_calculation(double x1, double x2, double y1, double y2, double x3)
{
    double y = ((y2 - y1) / (x2 - x1)) * (x3 - x1) + y1;
    return y;
}

double angleToZLength(double l_knee)
{
    l_knee = l_knee / 2;
    return 2 * m_to_m * sin((l_knee) * M_PI / 180);
}

double positionToAngle(double Position)
{
    return (double)((Position * 360) / 1024);

}


int angleToPosition(double angle)
{
    return (int)((angle * 1024) / 360);
}

void init_save()
{
    int position[22] = {0, };

    std::ifstream is;

    is.open("/home/robit/catkin_ws/src/inverse_kinematics/src/motion/walk_init");

    for(int i=0; i<23; i++)
        is >> position[i];

    is.close();


    for(int DXL_ID = 0; DXL_ID <= 22; DXL_ID++)
    {
//        std::cout << "position : " << position[DXL_ID] << std::endl;
        g_DXL_ID_position[DXL_ID] = position[DXL_ID];
        g_DXL_ID_INIT_position[DXL_ID] = position[DXL_ID];
        std::cout << "g_DXL_ID_INIT_position[" << (int)DXL_ID << "] = " << g_DXL_ID_INIT_position[DXL_ID] << std::endl;
    }
}



void kinematics_calculation1(Info_kinematics temp, double leg_p, double leg_r, int aaa)
{
    double p_z;
    double p_z_f2;
    double beta;
    double l_theta1;
    double l_theta2;
    double l_theta3;
    double l_theta4;
    double l_theta5;
    double l_theta6;
    double l_theta7;
    double l_theta8;

    double l_theta1_L;
    double l_theta2_L;
    double l_theta3_L;
    double l_theta4_L;
    double l_theta5_L;
    double l_theta6_L;

    double l_theta1_R;
    double l_theta2_R;
    double l_theta3_R;
    double l_theta4_R;
    double l_theta5_R;
    double l_theta6_R;

    if(Start_flag < startCnt)
    {
        theta_cnt = 25;
        Y_DEFAULT = 25;//10+10*Start_flag/startCnt;
        //temp.X_length = 20*Start_flag/startCnt;
        temp.X_length = 10+(60*Start_flag/startCnt);
        temp.L_yaw = 0;
        temp.Y_length = 0;//10;//-10
        temp.Z_length = 120-(10*Start_flag/startCnt);//108;//112-Start_flag;
        temp.PA = 0;
    }
    else
    {
        if(isRunningMode == true)
            theta_cnt = runningThetaCnt;
        else
            theta_cnt = defaultThetaCnt;

        Y_DEFAULT = swing;//Running : 10
        temp.Z_length = temp.Z_length;
        temp.PA = 0;
    }


    if(aaa == 1)
    {
        //temp.PA = -10.0;

        temp.Z_length = angleToZLength(temp.Z_length);

        p_z = sqrt((temp.Y_length * temp.Y_length) + (temp.Z_length * temp.Z_length));
        l_theta2 = acos(((p_z * p_z) + (temp.X_length * temp.X_length)) / (2 * m_to_m * m_to_m) - 1);
        beta = sqrt((1 + cos(l_theta2)) / 2);
        l_theta2 = acos(((p_z * p_z) + (temp.X_length * temp.X_length)) / (2 * m_to_m * m_to_m) - 1) * M_1_PI * 180;
        l_theta1 = (((2 * asin(temp.X_length / (2 * m_to_m * beta))) * M_1_PI * 180) + l_theta2) / 2;
        l_theta3 = (l_theta2 - ((2 * asin(temp.X_length / (2 * m_to_m * beta))) * M_1_PI * 180)) / 2 + leg_p;
        l_theta4 = asin(temp.Y_length / p_z) * M_1_PI * 180;
        l_theta5 = asin(temp.Y_length / p_z) * M_1_PI * 180 + leg_r;

        /*
         p_z  = sqrt((temp.Y_length * temp.Y_length)+(temp.Z_length * temp.Z_length));
         l_theta2 = acos(((p_z*p_z)+(temp.X_length * temp.X_length))/(2*m_to_m*m_to_m) - 1);
         beta = sqrt((1+cos(l_theta2))/2);
         l_theta2 = acos(((p_z*p_z)+(temp.X_length * temp.X_length))/(2*m_to_m*m_to_m) - 1)*M_1_PI*180;
         l_theta1 = ((((2*asin(temp.X_length/(2*m_to_m*beta)))*M_1_PI*180) + l_theta2)/2)*2;
         l_theta3 = (l_theta2 - (l_theta2 - ((2*asin(temp.X_length/(2*m_to_m*beta)))*M_1_PI*180))/2)*2 + leg_p;
         l_theta2 = acos(((p_z*p_z)+(temp.X_length * temp.X_length))/(2*m_to_m*m_to_m) - 1)*M_1_PI*180*2;
         l_theta4 = asin(temp.Y_length/p_z)*M_1_PI*180;
         l_theta5 = asin(temp.Y_length/p_z)*M_1_PI*180 + leg_r;
         */

        for(int i = 0; i < 23; i++)
        {
            g_DXL_ID_position[i] = g_DXL_ID_INIT_position[i];
        }

//        g_DXL_ID_position[14] += angleToPosition(l_theta1 + temp.PA);					//left_pitch	+
//        g_DXL_ID_position[16] -= angleToPosition(l_theta2);					//left_pitch
//        g_DXL_ID_position[18] -= angleToPosition(l_theta3);					//left_pitch	-
//        g_DXL_ID_position[10] += angleToPosition(l_theta4);					//left_roll		+
//        g_DXL_ID_position[20] += angleToPosition(l_theta5);					//left_roll

//        g_DXL_ID_position[15] -= angleToPosition(l_theta1 + temp.PA);					//right_pitch	-
//        g_DXL_ID_position[17] += angleToPosition(l_theta2);					//right_pitch
//        g_DXL_ID_position[19] += angleToPosition(l_theta3);					//right_pitch	+
//        g_DXL_ID_position[11] -= angleToPosition(l_theta4);					//right_roll	-
//        g_DXL_ID_position[21] -= angleToPosition(l_theta5);					//right_roll

        g_DXL_ID_position[14] -= angleToPosition(l_theta1 + temp.PA + ankle_14);					//left_pitch	+
        g_DXL_ID_position[16] += angleToPosition(l_theta2);					//left_pitch
        g_DXL_ID_position[18] += angleToPosition(l_theta3/* + ankle_18*/);					//left_pitch	-
        g_DXL_ID_position[10] += angleToPosition(l_theta4);					//left_roll		+
        g_DXL_ID_position[20] += angleToPosition(l_theta5/* + ankle_20*/);					//left_roll

        g_DXL_ID_position[15] += angleToPosition(l_theta1 + temp.PA + ankle_15);					//right_pitch	-
        g_DXL_ID_position[17] -= angleToPosition(l_theta2);					//right_pitch
        g_DXL_ID_position[19] -= angleToPosition(l_theta3/* + ankle_19*/);					//right_pitch	+
        g_DXL_ID_position[11] -= angleToPosition(l_theta4);					//right_roll	-
        g_DXL_ID_position[21] -= angleToPosition(l_theta5/* + ankle_21*/);					//right_roll

        for(int i = 0; i < 23; i++)
        {
          std::cout<<"g_DXL_ID_position["<<i<<"]"<<g_DXL_ID_position[i]<<std::endl;
        }

    }

    else if(aaa == 2)
    {
        double p_s_r;
        double p_s_l;

        double shoulder_r = 20;
        double shoulder_l = 20;

        double p_x_l;
        double p_x_r;
        double p_y_l;
        double p_y_r;
        double p_z_l;
        double p_z_r;
        double p_arm_r = 0;
        double p_arm_l = 0;
        double beta_l;
        double beta_r;
        double leg_yaw_l;
        double leg_yaw_r;
        static int Y = 1;
        static int X1 = 0;
        static int X2 = 0;
        double temp_y_length = 0;

        if(Start_flag < startCnt)
            temp.L_yaw = 0;

        temp.R_yaw = temp.L_yaw;
        temp.Z_length = angleToZLength(temp.Z_length);
        p_z_f2 = angleToZLength(g_KneeAngle) - temp.Z_length;

        g_theta++;
        temp_y_length = temp.Y_length;
        temp.Y_length = Only_Plus2(temp.Y_length);

        //std::cout << "temp.X_length = " << temp.X_length << std::endl;
        //std::cout << "temp.Y_length = " << temp.Y_length << std::endl;
        //std::cout << "temp.Z_length = " << temp.Z_length << std::endl;
        //std::cout << "temp.L_yaw = " << temp.L_yaw << std::endl;




//        if(Leg == 0)
//        {
//          p_s_r = shoulder_r * sin(M_PI * g_theta / theta_cnt);
//          p_s_l = -shoulder_l * sin(M_PI * g_theta / theta_cnt);
//        }
//        else if(Leg == 1)
//        {
//          p_s_r = -shoulder_r * sin(M_PI * g_theta / theta_cnt);
//          p_s_l = shoulder_l * sin(M_PI * g_theta / theta_cnt);
//        }

//        if(Leg == 0)
//        {
//          p_waist = 15*sin(M_PI * g_theta / theta_cnt);
//        }
//        else if(Leg == 1)
//        {
//          p_waist = -15*sin(M_PI * g_theta / theta_cnt);
//        }





        if(Leg == 0)
        {	// right

            /////////////////////////////////////////////////////////////////
            //////////////////////		Y_Pattern		/////////////////////
            /////////////////////////////////////////////////////////////////
            if(Start_flag <-10)//< startCnt)
            {
                p_y_r = (Start_flag/startCnt) * Y_DEFAULT * sin(M_PI * (g_theta / theta_cnt));
                p_y_l = -(Start_flag/startCnt) * Y_DEFAULT * sin(M_PI * (g_theta / theta_cnt));
            }
            else
            {
                if(temp_y_length > 0.1)
                {
                    if(g_theta >= 0 && g_theta < theta_cnt / 2)
                    {
                        p_y_l = -Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                        p_y_r = Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                    }
                    else if(g_theta >= theta_cnt / 2 && g_theta <= theta_cnt)
                    {
                        /*
                         if(g_theta >= theta_cnt*0.75)
                         p_y_l = temp.Y_length*cos(2*M_PI*g_theta/theta_cnt);
                         else
                         p_y_l = Y_DEFAULT*cos(2*M_PI*g_theta/theta_cnt);
                         */
                        p_y_l = -Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                        p_y_r = Y_DEFAULT + (Y_DEFAULT - temp.Y_length) * cos(M_PI * g_theta / theta_cnt);				//graph_calculation(theta_cnt/2,theta_cnt,Y_DEFAULT,temp.Y_length,g_theta);
                    }
                }
                else if(temp_y_length < -0.1)
                {

                    if(g_theta >= 0 && g_theta < theta_cnt / 2)
                    {
                        /*
                         if(g_theta >= theta_cnt*0.25)
                         p_y_l = Y_DEFAULT*cos(2*M_PI*g_theta/theta_cnt);
                         else
                         p_y_l = temp.Y_length*cos(2*M_PI*g_theta/theta_cnt);
                         */
                        p_y_l = temp.Y_length - (Y_DEFAULT + temp.Y_length) * sin(M_PI * g_theta / theta_cnt);
                        p_y_r = Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);				//graph_calculation(0,theta_cnt/2,temp.Y_length,Y_DEFAULT,g_theta);
                    }
                    else if(g_theta >= theta_cnt / 2 && g_theta <= theta_cnt)
                    {
                        p_y_l = -Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                        p_y_r = Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                    }
                }
                else
                {
                    p_y_r = Y_DEFAULT * sin(M_PI * (g_theta / theta_cnt));
                    p_y_l = -Y_DEFAULT * sin(M_PI * (g_theta / theta_cnt));
                }
            }

            if(Start_flag < startCnt)
            {
                leg_yaw_l = 0;
                leg_yaw_r = 0;
                //==============================================
                p_s_l = 0;
                p_s_r = 0;
            }
            else if(temp.L_yaw > 0)
            {
                if(g_theta >= 0 && g_theta < theta_cnt / 2)
                {
                    leg_yaw_l = -temp.L_yaw * sin(M_PI * g_theta / theta_cnt);
                    leg_yaw_r = temp.L_yaw * sin(M_PI * g_theta / theta_cnt);
                }
                else if(g_theta >= theta_cnt / 2 && g_theta <= theta_cnt)
                {
                    leg_yaw_l = -temp.L_yaw;
                    leg_yaw_r = temp.L_yaw;
                }

            }
            else if(temp.L_yaw < 0)
            {
                //std::cout << " !!" << std::endl;
                if(g_theta >= 0 && g_theta < theta_cnt / 4)
                {
                    leg_yaw_l = temp.L_yaw;
                    leg_yaw_r = -temp.L_yaw;
                }
                else if(g_theta >= theta_cnt / 4 && g_theta <= theta_cnt / 2)
                {
                    leg_yaw_l = temp.L_yaw * sin(2 * M_PI * g_theta / theta_cnt);
                    leg_yaw_r = -temp.L_yaw * sin(2 * M_PI * g_theta / theta_cnt);
                }
                else if(g_theta >= theta_cnt / 2 && g_theta <= theta_cnt)
                {
                    leg_yaw_l = 0;
                    leg_yaw_r = 0;
                }

                /*
                 if(g_theta >= 0 && g_theta < theta_cnt/2){
                 leg_yaw_l = temp.L_yaw*cos(M_PI*g_theta/theta_cnt);
                 leg_yaw_r = -temp.L_yaw*cos(M_PI*g_theta/theta_cnt);
                 }
                 else if(g_theta >= theta_cnt/2 && g_theta <= theta_cnt){
                 leg_yaw_l = 0;
                 leg_yaw_r = 0;
                 }*/

            }
            else
            {
                leg_yaw_l = 0;
                leg_yaw_r = 0;
            }
        }

        else if(Leg)
        {
            if(Start_flag < -10)//startCnt)
            {
                p_y_r = -Y_DEFAULT*(Start_flag/startCnt) * sin(M_PI * (g_theta / theta_cnt));
                p_y_l = Y_DEFAULT*(Start_flag/startCnt) * sin(M_PI * (g_theta / theta_cnt));
            }
            else
            {
                if(temp_y_length > 0)
                {
                    if(g_theta >= 0 && g_theta < theta_cnt / 2)
                    {
                        p_y_l = Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);				//graph_calculation(0,theta_cnt/2,temp.Y_length,Y_DEFAULT,g_theta);
                        /*
                         if(g_theta >= theta_cnt*0.25)
                         p_y_r = Y_DEFAULT*cos(2*M_PI*g_theta/theta_cnt);
                         else
                         p_y_r = temp.Y_length*cos(2*M_PI*g_theta/theta_cnt);
                         */
                        p_y_r = temp.Y_length - (Y_DEFAULT + temp.Y_length) * sin(M_PI * g_theta / theta_cnt);
                    }
                    else if(g_theta >= theta_cnt / 2 && g_theta <= theta_cnt)
                    {
                        p_y_l = Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                        p_y_r = -Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                    }
                }
                else if(temp_y_length < 0)
                {
                    if(g_theta >= 0 && g_theta < theta_cnt / 2)
                    {
                        p_y_l = Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                        p_y_r = -Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                    }
                    else if(g_theta >= theta_cnt / 2 && g_theta <= theta_cnt)
                    {
                        p_y_l = Y_DEFAULT + (Y_DEFAULT - temp.Y_length) * cos(M_PI * g_theta / theta_cnt);				//graph_calculation(theta_cnt/2,theta_cnt,Y_DEFAULT,temp.Y_length,g_theta);
                        /*if(g_theta >= theta_cnt*0.75)
                         p_y_r = temp.Y_length*cos(2*M_PI*g_theta/theta_cnt);
                         else
                         p_y_r = Y_DEFAULT*cos(2*M_PI*g_theta/theta_cnt);
                         */
                        p_y_r = -Y_DEFAULT * sin(M_PI * g_theta / theta_cnt);
                    }
                }
                else
                {
                    p_y_r = -Y_DEFAULT * sin(M_PI * (g_theta / theta_cnt));
                    p_y_l = Y_DEFAULT * sin(M_PI * (g_theta / theta_cnt));
                }

            }
            if(Start_flag < startCnt)
            {
                leg_yaw_l = 0;
                leg_yaw_r = 0;
                //==============================================
                p_s_l = 0;
                p_s_r = 0;
            }
            else if(temp.L_yaw > 0)
            {
                if(g_theta >= 0 && g_theta < theta_cnt / 4)
                {
                    leg_yaw_l = -temp.L_yaw;
                    leg_yaw_r = temp.L_yaw;
                }
                else if(g_theta >= theta_cnt / 4 && g_theta <= theta_cnt / 2)
                {
                    leg_yaw_l = -temp.L_yaw * sin(2 * M_PI * g_theta / theta_cnt);
                    leg_yaw_r = temp.L_yaw * sin(2 * M_PI * g_theta / theta_cnt);
                }
                else if(g_theta >= theta_cnt / 2 && g_theta <= theta_cnt)
                {
                    leg_yaw_l = 0;
                    leg_yaw_r = 0;
                }
            }
            else if(temp.L_yaw < 0)
            {
                if(g_theta >= 0 && g_theta < theta_cnt / 2)
                {
                    leg_yaw_l = temp.L_yaw * sin(M_PI * g_theta / theta_cnt);
                    leg_yaw_r = -temp.L_yaw * sin(M_PI * g_theta / theta_cnt);
                }
                else if(g_theta >= theta_cnt / 2 && g_theta <= theta_cnt)
                {
                    leg_yaw_l = temp.L_yaw;
                    leg_yaw_r = -temp.L_yaw;
                }
            }
            else
            {
                leg_yaw_l = 0;
                leg_yaw_r = 0;
            }
        }

        p_y_r += df_yr;
        p_y_l += df_yl;

        p_z_r = sqrt((p_y_r * p_y_r) + (angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle)));
        p_z_l = sqrt((p_y_l * p_y_l) + (angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle)));

        //if(g_theta > theta_cnt/6){
        if(Leg == 0)  // Right Leg
        {
            if(Start_flag < startCnt)
            {
                p_z_r = sqrt((p_y_r * p_y_r) + (angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle)))
                            - (angleToZLength(g_KneeAngle) - angleToZLength(g_KneeAngle - 20*(Start_flag)/startCnt)) * (1 - cos(2 * M_PI * (g_theta / theta_cnt)));
                p_x_r = (START_X) * (-cos(M_PI * (g_theta / theta_cnt)));
                p_x_l = (START_X) * (cos(M_PI * (g_theta / theta_cnt)));
            }
            else
            {
                p_z_r = sqrt((p_y_r * p_y_r) + (angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle))) - p_z_f2 * (1 - cos(2 * M_PI * (g_theta / theta_cnt)));
                p_x_r = (temp.X_length / 2) * (-cos(M_PI * (g_theta / theta_cnt)));
                p_x_l = (temp.X_length / 2) * (cos(M_PI * (g_theta / theta_cnt)));
            }
        }

        else if(Leg)  // Left Leg
        {
            if(Start_flag < startCnt)
            {
                p_z_l = sqrt((p_y_l * p_y_l) + (angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle)))
                            - (angleToZLength(g_KneeAngle) - angleToZLength(g_KneeAngle - 20*(Start_flag)/startCnt)) * (1 - cos(2 * M_PI * (g_theta / theta_cnt)));
                p_x_r = (START_X) * (cos(M_PI * (g_theta / theta_cnt)));
                p_x_l = (START_X) * (-cos(M_PI * (g_theta / theta_cnt)));
            }
            else
            {
                p_z_l = sqrt((p_y_l * p_y_l) + (angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle))) - p_z_f2 * (1 - cos(2 * M_PI * (g_theta / theta_cnt)));
                p_x_r = (temp.X_length / 2) * (cos(M_PI * (g_theta / theta_cnt)));
                p_x_l = (temp.X_length / 2) * (-cos(M_PI * (g_theta / theta_cnt)));

            }
        }

        //}



        if(g_theta == theta_cnt)
        {
            p_y_r = df_yr;
            p_y_l = df_yl;
            p_z_r = sqrt((p_y_r * p_y_r) + (angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle)));
            p_z_l = sqrt((p_y_l * p_y_l) + (angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle)));
            Y = -Y;
            Leg ^= 1;
            g_theta = 0;
            Start_flag++;
            theta_count++;
            //==============================================
            p_s_l = 0;
            p_s_r = 0;
        }



        p_z_r += df_zr;
        p_z_l += df_zl;

//        std::cout << "theta = " << g_theta << std::endl;
//        std::cout << "Leg = " << Leg << std::endl;
//        std::cout << "temp.X_length = " << temp.X_length << std::endl;
        //std::cout << "p_y_r = " << p_y_r << std::endl;
        //std::cout << "leg_yaw_r = " << leg_yaw_r << std::endl;

        /*
         p_x_l = temp.X_length * sin(g_theta);
         p_x_r = temp.X_length * sin(g_theta+M_PI);



         if(temp.Y_length == 0){
         p_y_l = -10 * cosKB(g_theta+M_PI,2);
         p_y_r = -10 * cosKB(g_theta,2);
         }
         else if(temp.Y_length < 0){
         if(g_theta >= 0 && g_theta < M_PI_2){
         p_y_l = -temp.Y_length;
         p_y_r = temp.Y_length*cos(2*g_theta);
         }
         else if(g_theta >= M_PI_2 && g_theta < M_PI){
         p_y_l = temp.Y_length*cos(2*g_theta);
         p_y_r = -temp.Y_length;
         }
         else if(g_theta >= M_PI && g_theta < M_PI*2){
         p_y_l = -temp.Y_length * cos(g_theta);
         p_y_r = temp.Y_length * cos(g_theta);
         }
         }
         else if(temp.Y_length > 0){
         if(g_theta >= 0 && g_theta < M_PI){
         p_y_l = temp.Y_length * cos(g_theta);
         p_y_r = temp.Y_length * cos(g_theta+M_PI);
         }
         else if(g_theta >= M_PI && g_theta < M_PI+M_PI_2){
         p_y_l = temp.Y_length * -cos(g_theta*2);
         p_y_r = temp.Y_length;
         }
         else if(g_theta >= M_PI+M_PI_2 && g_theta < M_PI*2){
         p_y_l = temp.Y_length;
         p_y_r = temp.Y_length * -cos(g_theta*2);
         }
         }



         p_z_l  = sqrt((temp.Y_length * temp.Y_length)+(angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle))) - p_z_f2 * Only_Plus(cosKB(g_theta,1));//(1+cosKB(g_theta,2))/2
         p_z_r  = sqrt((temp.Y_length * temp.Y_length)+(angleToZLength(g_KneeAngle) * angleToZLength(g_KneeAngle))) - p_z_f2 * Only_Plus(cosKB(g_theta+M_PI,1));//(1+cosKB(g_theta + M_PI,1))/2
         */

        l_theta2_L = acos(((p_z_l * p_z_l) + (p_x_l * p_x_l)) / (2 * m_to_m * m_to_m) - 1);
        l_theta2_R = acos(((p_z_r * p_z_r) + (p_x_r * p_x_r)) / (2 * m_to_m * m_to_m) - 1);

        beta_l = sqrt((1 + cos(l_theta2_L)) / 2);
        beta_r = sqrt((1 + cos(l_theta2_R)) / 2);

        l_theta2_L = acos(((p_z_l * p_z_l) + (p_x_l * p_x_l)) / (2 * m_to_m * m_to_m) - 1) * M_1_PI * 180;
        l_theta2_R = acos(((p_z_r * p_z_r) + (p_x_r * p_x_r)) / (2 * m_to_m * m_to_m) - 1) * M_1_PI * 180;

        l_theta1_L = (((2 * asin(p_x_l / (2 * m_to_m * beta_l))) * M_1_PI * 180) + l_theta2_L) / 2;
        l_theta1_R = (((2 * asin(p_x_r / (2 * m_to_m * beta_r))) * M_1_PI * 180) + l_theta2_R) / 2;

        l_theta3_L = (l_theta2_L - ((2 * asin(p_x_l / (2 * m_to_m * beta_l))) * M_1_PI * 180)) / 2 + leg_p;
        l_theta3_R = (l_theta2_R - ((2 * asin(p_x_r / (2 * m_to_m * beta_r))) * M_1_PI * 180)) / 2 + leg_p;

        l_theta4_L = atan(p_y_l / p_z_l) * M_1_PI * 180;
        l_theta4_R = atan(p_y_r / p_z_r) * M_1_PI * 180;

        l_theta5_L = atan(p_y_l / p_z_l) * M_1_PI * 180 + leg_r;
        l_theta5_R = atan(p_y_r / p_z_r) * M_1_PI * 180 + leg_r;

        l_theta6_L = leg_yaw_l;
        l_theta6_R = leg_yaw_r;

        l_theta7 = temp.PA;
        l_theta8 = temp.RA;

        /*
         l_theta2_L = acos(((p_z_l*p_z_l)+(p_x_l * p_x_l))/(2*m_to_m*m_to_m) - 1);
         l_theta2_R = acos(((p_z_r*p_z_r)+(p_x_r * p_x_r))/(2*m_to_m*m_to_m) - 1);

         beta_l = sqrt((1+cos(l_theta2_L))/2);
         beta_r = sqrt((1+cos(l_theta2_R))/2);

         l_theta2_L = acos(((p_z_l*p_z_l)+(p_x_l * p_x_l))/(2*m_to_m*m_to_m) - 1)*M_1_PI*180;
         l_theta2_R = acos(((p_z_r*p_z_r)+(p_x_r * p_x_r))/(2*m_to_m*m_to_m) - 1)*M_1_PI*180;

         l_theta1_L = (((2*asin(p_x_l/(2*m_to_m*beta_l)))*M_1_PI*180) + l_theta2_L);
         l_theta1_R = (((2*asin(p_x_r/(2*m_to_m*beta_r)))*M_1_PI*180) + l_theta2_R);

         l_theta3_L = (l_theta2_L - (l_theta2_L - ((2*asin(p_x_l/(2*m_to_m*beta_l)))*M_1_PI*180))/2 + leg_p)*2;
         l_theta3_R = (l_theta2_R - (l_theta2_R - ((2*asin(p_x_r/(2*m_to_m*beta_r)))*M_1_PI*180))/2 + leg_p)*2;

         l_theta2_L = acos(((p_z_l*p_z_l)+(p_x_l * p_x_l))/(2*m_to_m*m_to_m) - 1)*M_1_PI*180*2;
         l_theta2_R = acos(((p_z_r*p_z_r)+(p_x_r * p_x_r))/(2*m_to_m*m_to_m) - 1)*M_1_PI*180*2;


         l_theta4_L = atan(p_y_l/p_z_l)*M_1_PI*180;
         l_theta4_R = atan(p_y_r/p_z_r)*M_1_PI*180;

         l_theta5_L = atan(p_y_l/p_z_l)*M_1_PI*180 + leg_r;
         l_theta5_R = atan(p_y_r/p_z_r)*M_1_PI*180 + leg_r;

         l_theta6_L = leg_yaw_l;
         l_theta6_R = leg_yaw_r;

         l_theta7 = temp.PA;
         l_theta8 = temp.RA;
         */

        /*
         g_theta += M_PI/32;//0.13;
         theta_count ++;

         if(theta_count == 64){
         g_theta = 0;
         theta_count = 0;
         }
         */
        for(int i = 0; i < 23; i++)
        {
            g_DXL_ID_position[i] = g_DXL_ID_INIT_position[i];
        }

          g_DXL_ID_position[0] += angleToPosition(p_s_l);							//righyt shoulder
          g_DXL_ID_position[1] -= angleToPosition(p_s_r);							//left shoulder

          g_DXL_ID_position[12] += angleToPosition(l_theta6_L + ankle_12);							//left_yaw
          g_DXL_ID_position[13] += angleToPosition(l_theta6_R + ankle_13);							//right_yaw

          g_DXL_ID_position[14] -= angleToPosition((l_theta1_L) + l_theta7 + ankle_14);				//left_pitch
          g_DXL_ID_position[16] += angleToPosition(l_theta2_L);				// - l_theta7 );							//left_pitch
          g_DXL_ID_position[18] += angleToPosition((l_theta3_L)+ankle_18);				// - l_theta7*0.7);							//left_pitch
          g_DXL_ID_position[10] += angleToPosition(l_theta4_L);				//left_roll
          g_DXL_ID_position[20] += angleToPosition(l_theta5_L + l_theta8+ankle_20);				//left_roll   ++

          g_DXL_ID_position[15] += angleToPosition((l_theta1_R) + l_theta7 +ankle_15);				//right_pitch
          g_DXL_ID_position[17] -= angleToPosition(l_theta2_R);				// - l_theta7);							//right_pitch
          g_DXL_ID_position[19] -= angleToPosition((l_theta3_R)+ankle_19);				// - l_theta7*0.7);							//right_pitch
          g_DXL_ID_position[11] -= angleToPosition(l_theta4_R);				//right_roll
          g_DXL_ID_position[21] -= angleToPosition(l_theta5_R + l_theta8+ankle_21);				//right_roll  --

          g_DXL_ID_position[22] += angleToPosition(p_waist);				//right_roll  --


//          g_DXL_ID_position[30] += angleToPosition(-p_waist);				//right_roll  --

//          std::cout<<"ankle18 = "<<ankle_18<<std::endl;



        if(g_DXL_ID_position[6] < g_DXL_ID_INIT_position[6])
            g_DXL_ID_position[6] = g_DXL_ID_INIT_position[6];

        if(g_DXL_ID_position[7] > g_DXL_ID_INIT_position[7])
            g_DXL_ID_position[7] = g_DXL_ID_INIT_position[7];

    }

}

double Only_Plus(double thetaz)
{
    if(thetaz >= 0)
        return thetaz;
    else
        return 0;
}

double Only_Plus2(double thetaz)
{
    if(thetaz >= 0)
        return thetaz;
    else
        return thetaz * (-1);
}

double cosKB(double theta1, float rate)
{
    if(rate * cos(theta1) > 1)
        return 1;

    else if(rate * cos(theta1) < -1)
        return -1;

    else
        return rate * cos(theta1);
}

