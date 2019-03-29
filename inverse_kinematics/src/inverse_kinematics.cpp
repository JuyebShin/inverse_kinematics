/*
 * inverse_kinematics.cpp
 *
 *  Created on: Aug 12, 2015
 *      Author:
 */

#include "ros/ros.h"
#include <iostream>
#include <string>
#include "../include/inverse_kinematics/info_kinematics_msg.h"
#include "../include/inverse_kinematics/inverse_kinematics_coordinate_msg.h"
#include "../include/inverse_kinematics/motor_dxl_msg.h"
#include "../include/inverse_kinematics/Motor_msg.h"
#include "../src/Header/Moving_point.h"
#include "../src/Header/Play_Kinematics.h"
#include "../src/Header/Motion_msg.h"
#include "../include/inverse_kinematics/set_kinematics.h"


#include <fstream>
#include "../src/Header/pid_control_float.h"
#include <cmath>

//#include <QString>

#define MXALL 2
#define RX_28 1

#define ALL_BODY 0
#define LOW_BODY 10

using namespace std;

void init_ready3(double ms_time, bool starting_flag);
void init_ready(int body, int limit);
void init_ready2(Info_kinematics KM);
void init_ready_CW(void);

// -- msg --
motor_dxl::motor_dxl_msg dxMsg1;
Motion1::Motion_msg MotionMsg;
inverse_kinematics::inverse_kinematics_coordinate_msg IKCMsg;
set_walk::set_kinematics setmsg;

ros::Publisher mcuPub;
ros::Publisher pub_stand;
ros::Publisher pub_Motion;
ros::Publisher pub_coordinate;
ros::Subscriber sub;
ros::Subscriber sub_setwalk;

Info_kinematics Info_kinematics_, temp_IK, new_IK;


Moving_point ZMP_point(0, 0, 0);

PID PITCH;
PID PITCH_STEP;
PID ROLL;

//*****************************************************************************
bool isRunningMode = false;
int runningStartCnt = 12;//12   //15  //running 15  //basket 25
int defaultStartCnt = 5; //13         //running 8  //basket 13    //12
int startCnt = defaultStartCnt;
int runningThetaCnt = 29;//26
int defaultThetaCnt = 29;//30

int Xmoved;
//int runningThetaCnt = 35;//26
//int defaultThetaCnt = 35;//30

double g_KneeAngle = 120;//init angle

int df_yr = 0;
int df_yl = 8;

int df_zr = 0;
int df_zl = 0;

int ankle_12 = 0;
int ankle_13 = 0;

int ankle_14 = 6;
int ankle_15 = 6;

int ankle_18 = 0;
int ankle_19 = 0;
int ankle_20 = 0;
int ankle_21 = 0;

int swing = 15;
//*****************************************************************************

int stop_flag = 2;


double Y_DEFAULT = 10;


double g_theta;

int fall_count = 0;
int fall_flag = 0;

int g_DXL_ID_position[100];
int g_DXL_ID_INIT_position[100];
int g_DXL_ID_Past_position[23];
double speed[23];
int theta_count;
int theta_cnt = defaultThetaCnt;
int Start_flag;

double Temp_Q[20];

int Step_counter = 0;
int Step_counter2 = 0;
int flag;


int g_Motor[24] = {
        MXALL,                                  // 0
        MXALL,									// 1
        MXALL,									// 2
        MXALL,									// 3
        MXALL,									// 4
        MXALL,									// 5
        MXALL,									// 6
        MXALL,									// 7
        MXALL,									// 8
        MXALL,									// 9
        MXALL,									// 10
        MXALL,									// 11
        MXALL,									// 12
        MXALL,									// 13
        MXALL,									// 14
        MXALL,									// 15
        MXALL,									// 16
        MXALL,									// 17
        MXALL,									// 18
        MXALL,									// 19
        MXALL,									// 20
        MXALL,									// 21
        MXALL,									// 22
        MXALL,									// 23
};

double temp_X, temp_X2;
double temp_Z, temp_flag;

int Accelcase = 0;
void KinematicsCallback(const inverse_kinematics::info_kinematics_msg::ConstPtr& msg)
{

    Info_kinematics_.X_length       = msg->X_length;
    Info_kinematics_.Y_length       = msg->Y_length;
    Info_kinematics_.Z_length       = msg->Z_length;
    Info_kinematics_.R_yaw          = msg->R_yaw;
    Info_kinematics_.L_yaw          = msg->L_yaw;
    Info_kinematics_.flag           = msg->flag;
    Info_kinematics_.KM_flag        = msg->KM_flag;
    Info_kinematics_.Side_flag      = msg->Side_flag;
    Info_kinematics_.KM_flag2       = msg->KM_flag2;
    Info_kinematics_.Step_counter   = msg->Step_counter;
    Info_kinematics_.PA             = msg->PA;
    temp_Z = msg->Z_length;
    temp_X2 = msg->X_length;
    //cout << msg->Z_length << endl;
    for(int i = 0; i < msg->KM_flag; i++)
    {
        Info_kinematics_.Using_Motor_Id[i] = msg->Using_Motor_id[i];
    }

    if(temp_IK.X_length + 6 < Info_kinematics_.X_length && Start_flag > START_CNT)
    {
        //cout << "Accelcase1" << endl;
        Accelcase = 1;
        //temp_X = temp_IK.X_length;
    }
    else if(temp_IK.X_length - 10 > Info_kinematics_.X_length && Start_flag > START_CNT)
    {
        //cout << "Accelcase2" << endl;
        Accelcase = 2;
        //temp_X = temp_IK.X_length;
    }
    else
    {
        Accelcase = 0;
        //cout << "Accelcase0" << endl;
    }

    if(Start_flag <= START_CNT)
        Info_kinematics_.X_length = DEFAULT_X;

    if(msg->flag == 0 && temp_flag != 0)
        stop_flag = 0;

    if(Info_kinematics_.L_yaw > 20)
    {
        Info_kinematics_.L_yaw = 20;
    }
    else if(Info_kinematics_.L_yaw < -20)
        Info_kinematics_.L_yaw = -20;

    flag = 1;
    temp_flag = msg->flag;

//    if(Info_kinematics_.X_length > 50)
//    {
//      ankle_14 = Info_kinematics_.X_length*0.09;
//      ankle_15 = Info_kinematics_.X_length*0.09;
//    }

//    if(ankle_14 < 4 || ankle_15 < 4)
//    {
//      ankle_14 = 4;
//      ankle_15 = 4;
//    }

}




void set_walk_Callback(const set_walk::set_kinematics::ConstPtr& msg)
{
  Info_kinematics_.X_length       = msg->X_length;
  Info_kinematics_.Y_length       = msg->Y_length;
  Info_kinematics_.Z_length       = msg->Z_length;
  Info_kinematics_.R_yaw          = msg->R_yaw;
  Info_kinematics_.L_yaw          = msg->L_yaw;
  Info_kinematics_.flag           = msg->flag;
  Info_kinematics_.KM_flag        = msg->KM_flag;
  Info_kinematics_.Side_flag      = msg->Side_flag;
  Info_kinematics_.KM_flag2       = msg->KM_flag2;
  Info_kinematics_.Step_counter   = msg->Step_counter;
  Info_kinematics_.PA             = msg->PA;
  temp_Z = msg->Z_length;
  temp_X2 = msg->X_length;
  //cout << msg->Z_length << endl;
  for(int i = 0; i < msg->KM_flag; i++)
  {
      Info_kinematics_.Using_Motor_Id[i] = msg->Using_Motor_id[i];
  }

  if(temp_IK.X_length + 6 < Info_kinematics_.X_length && Start_flag > START_CNT)
  {
      //cout << "Accelcase1" << endl;
      Accelcase = 1;
      //temp_X = temp_IK.X_length;
  }
  else if(temp_IK.X_length - 10 > Info_kinematics_.X_length && Start_flag > START_CNT)
  {
      //cout << "Accelcase2" << endl;
      Accelcase = 2;
      //temp_X = temp_IK.X_length;
  }
  else
  {
      Accelcase = 0;
      //cout << "Accelcase0" << endl;
  }

  if(Start_flag <= START_CNT)
      Info_kinematics_.X_length = DEFAULT_X;

  if(msg->flag == 0 && temp_flag != 0)
      stop_flag = 0;

  if(Info_kinematics_.L_yaw > 20)
  {
      Info_kinematics_.L_yaw = 20;
  }
  else if(Info_kinematics_.L_yaw < -20)
      Info_kinematics_.L_yaw = -20;

  flag = 1;
  temp_flag = msg->flag;

  df_yr = msg->df_yr;
  df_yl = msg->df_yl;

//  cout<<"df_yr = "<<df_yr<<endl;
//  cout<<"df_yl = "<<df_yl<<endl;

  df_zr = msg->init_ang_zr;
  df_zl = msg->init_ang_zl;

//  cout<<"df_zr = "<<df_zr<<endl;
//  cout<<"df_zl = "<<df_zl<<endl;

  ankle_18 = msg->ankle_18;
  ankle_19 = msg->ankle_19;
  ankle_20 = msg->ankle_20;
  ankle_21 = msg->ankle_21;

//  cout<<"ankle_18 = "<<ankle_18<<endl;
//  cout<<"ankle_19 = "<<ankle_19<<endl;

  defaultThetaCnt = msg->time;
  swing = msg->swing;

  ankle_14 = msg->ankle_14;
  ankle_15 = msg->ankle_15;
//    cout<<"ankle_14 = "<<ankle_14<<endl;
//    cout<<"ankle_15 = "<<ankle_15<<endl;

  ankle_12 = msg->ankle_12;
  ankle_13 = msg->ankle_13;
//    cout<<"ankle_12 = "<<ankle_12<<endl;
//    cout<<"ankle_13 = "<<ankle_13<<endl;

}

void parameter_save()
{
    std::ifstream is;

    is.open("/home/robit/catkin_ws/src/set_walk/work/no1");

       is >> Step_counter;
       is >> defaultThetaCnt;
       is >> swing;
       is >> df_yr;
       is >> df_yl;
       is >> df_zr;
       is >> df_zl;
       is >> ankle_12;
       is >> ankle_13;
       is >> ankle_14;
       is >> ankle_15;
       is >> ankle_18;
       is >> ankle_19;
       is >> ankle_20;
       is >> ankle_21;

//       cout<<ankle_20<<endl;
//       cout<<defaultThetaCnt<<endl;

    is.close();
}

bool Puflag = false;

int counterPu = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverse_kinematics");
    ros::NodeHandle n;

    //ros::Timer timer = n.createTimer(ros::Duration(0.1), StandUpTimer);

    ros::Rate r(100);

    new_IK.X_length = 0;
    new_IK.Y_length = 0;
    new_IK.Z_length = 100;

    if(argc > 1)
    {
        isRunningMode = true;
        theta_cnt = runningThetaCnt;
        startCnt = runningStartCnt;
    }

//    std::cout << "theta_cnt : " <<theta_cnt << std::endl;

    //cout << "adfef" << endl;

    mcuPub = n.advertise<serial_mcu::Motor_msg>("Dynamixel", 100);
    pub_Motion = n.advertise<Motion1::Motion_msg>("motion", 100);
    pub_coordinate = n.advertise<inverse_kinematics::inverse_kinematics_coordinate_msg>("kinematics_coordinate", 100);

    //cout << "*************** inverse kinematics!!!! ***************" << endl;

    sub = n.subscribe("kinematics", 100, KinematicsCallback);
    sub_setwalk = n.subscribe("set_walk",100,set_walk_Callback);

    init_save();
    parameter_save();
    sleep(1);

    for(int i = 0; i < 23; i++)
    {
        g_DXL_ID_Past_position[i] = g_DXL_ID_INIT_position[i];
    }

    Info_kinematics_.flag = 0;

    Info_kinematics_.Z_length = g_KneeAngle;									//g_KneeAngle);

    kinematics_calculation1(Info_kinematics_, 0, 0, 1);
    init_ready(ALL_BODY, 23);
    //g_DXL_ID_position

    for(int i = 0; i < 23; i++)
    {
        g_DXL_ID_Past_position[i] = g_DXL_ID_position[i];
    }

    sleep(1);

    //ofstream fout;
    //SerialPort imu("/dev/ttyACM0",115200,2,0);

    //////////////////////PID P,I,D gain value setting/////////////////////
    PITCH.kP = 0.6;
    PITCH.kD = 0.1;
    PITCH.kI = 0;
    PITCH.errorSumLimit = 0;

    PITCH_STEP.kP = 1;
    PITCH_STEP.kD = 0;
    PITCH_STEP.kI = 0;
    PITCH_STEP.errorSumLimit = 0;

    ROLL.kP = 0.5;

    while(ros::ok())
    {
        if(Accelcase == 1 && Start_flag > START_CNT)
        {
            Info_kinematics_.X_length = temp_X + 6;
        }
        else if(Accelcase == 2 && Start_flag > START_CNT)
        {
            Info_kinematics_.X_length = temp_X - 10;
        }

        if(g_theta == theta_cnt / 2 && Start_flag > START_CNT)
        {
            if(Accelcase == 1)
            {
                temp_X += 6;
                if(abs(temp_X2 - temp_X) < 6)
                    Accelcase = 0;
            }
            else if(Accelcase == 2)
            {
                temp_X -= 10;
                if(abs(temp_X2 - temp_X) < 10)
                    Accelcase = 0;
            }


            //new_IK = Info_kinematics_;

        }
        //cout << "Info_kinematics_.X_length = " << Info_kinematics_.X_length << endl;

        //////////////////////////////////////////kinematics mode//////////////////////////////////////////////////

        if(Info_kinematics_.flag == 0)						//IF don't want to play kinematics
        {

            if(g_theta == theta_cnt - 1)
            {
                if(stop_flag == 0)
                {
                    stop_flag = 1;
                }
                else
                {
                    stop_flag = 2;
                    memset(&Info_kinematics_, 0, sizeof(Info_kinematics_));
                    Info_kinematics_.X_length = DEFAULT_X;
                    Info_kinematics_.Y_length = DEFAULT_Y;
                    Info_kinematics_.Z_length = g_KneeAngle;
                    Info_kinematics_.L_yaw = 0;
                    Info_kinematics_.PA       = 0;
                    g_theta = 0;
                    Start_flag = 0;
                    theta_count = 0;
                    kinematics_calculation1(Info_kinematics_, 0, 0, 1);
                    Info_kinematics_.flag = 2;
                }
            }

            if(stop_flag == 0)
            {
                Info_kinematics_.PA = 0;
                Info_kinematics_.X_length = DEFAULT_X;
                Info_kinematics_.Y_length = DEFAULT_Y;
                Info_kinematics_.Z_length = 110;
                Info_kinematics_.L_yaw = 0;
                Info_kinematics_.PA       = 0;
                kinematics_calculation1(Info_kinematics_, 0, 0, 2);
                if(Info_kinematics_.KM_flag2 == 0)
//                    init_ready(ALL_BODY, 22);//init_ready(LOW_BODY, 22);
                    init_ready_CW();
                else if(Info_kinematics_.KM_flag2 == 1)
//                    init_ready(ALL_BODY, 22);//init_ready(LOW_BODY, 22);
                  init_ready_CW();

            }
            else if(stop_flag == 1)
            {
                Info_kinematics_.X_length = 0;
                Info_kinematics_.L_yaw = 0;
                Info_kinematics_.Z_length = 110;						//g_KneeAngle);
                Info_kinematics_.PA = 0;
                kinematics_calculation1(Info_kinematics_, 0, 0, 2);
                if(Info_kinematics_.KM_flag2 == 0)
//                    init_ready(ALL_BODY, 22);//init_ready(LOW_BODY, 22);
                  init_ready_CW();

                else if(Info_kinematics_.KM_flag2 == 1)
//                    init_ready(ALL_BODY, 22);//init_ready(LOW_BODY, 22);
                  init_ready_CW();
            }
            else if(stop_flag == 2)
            {
                Info_kinematics_.flag = 2;
            }
        }

        if(Info_kinematics_.flag == 1)														//Playing kinemtaics mode
        {
//            cout << "Info_kinematics_.X_length = " << Info_kinematics_.X_length << endl;
//            cout << "Info_kinematics_.YawYaw = " << Info_kinematics_.L_yaw << endl;
//            cout << "Info_kinematics_.Z_length = " << Info_kinematics_.Z_length << endl;

            stop_flag = 1;
            //if(Info_kinematics_.Side_flag == 0)
            kinematics_calculation1(/*Info_kinematics_*/Info_kinematics_, 0, 0, 2);
//			/new_IK
            //init_ready3(20,0);

            if(Info_kinematics_.KM_flag == 0)												//ALL motor playing mode
//                init_ready(ALL_BODY, 22);//init_ready(LOW_BODY, 22);
              init_ready_CW();

            else
                //Part motor playing mode
//                init_ready(ALL_BODY, 22);//init_ready(LOW_BODY, 23);
              init_ready_CW();


            //Info_kinematics_.flag = 2;
            //cin >> qwer;
        }

        if(Info_kinematics_.flag == 2)
        {
            g_theta = 0;
            Start_flag = 0;
            theta_count = 0;
        }
        if(Info_kinematics_.flag == 3)
        {
            g_theta = 0;
            Start_flag = 0;
            theta_count = 0;
            Info_kinematics_.X_length = 0;
            Info_kinematics_.Z_length = 120;			//g_KneeAngle);
            Info_kinematics_.Y_length = 0;
            kinematics_calculation1(Info_kinematics_, 0, 0, 1);
            init_ready(LOW_BODY, 23);//init_ready(LOW_BODY, 23);
        }

        for(int i = 0; i < 23; i++)
            g_DXL_ID_Past_position[i] = g_DXL_ID_position[i];

        if(g_theta == 0 && Info_kinematics_.flag == 1 && Start_flag > START_CNT)
        {
            static bool step = false;
            if(step)
            {
                IKCMsg.X_length = (Info_kinematics_.X_length - DEFAULT_X);
                IKCMsg.Y_length = (Info_kinematics_.Y_length - DEFAULT_Y);
                IKCMsg.YAW_length = (Info_kinematics_.L_yaw - DEFAULT_YAW);

                cout << "IKCMsg.X_length = " << IKCMsg.X_length << endl;
                cout << "IKCMsg.Y_length = " << IKCMsg.Y_length << endl;
                pub_coordinate.publish(IKCMsg);
            }

            step ^= 1;
        }
        temp_IK.X_length = Info_kinematics_.X_length;

        ros::spinOnce();

        r.sleep();
    }
    //fout.close();
    //imu.~SerialPort();
    return 0;
}

void init_ready(int body, int limit)
{
    serial_mcu::Motor_msg dxMsg;
    int i;

    for(i = body; i < limit; i++)
    {
        dxMsg.id.push_back(i);
        int pos = g_DXL_ID_position[i];
        dxMsg.position.push_back(pos << 2);

        dxMsg.speed.push_back(200/*speed[i]*2*/);
    }
    dxMsg.length = dxMsg.id.size();
    dxMsg.mode = 3;

    mcuPub.publish(dxMsg);
}

void init_ready_CW(void)
{
    serial_mcu::Motor_msg dxMsg;
    int i;

    for(i = 10; i < 23; i++)
    {
        dxMsg.id.push_back(i);
        int pos = g_DXL_ID_position[i];
        dxMsg.position.push_back(pos << 2);

        dxMsg.speed.push_back(400);
    }

    dxMsg.length = dxMsg.id.size();
    dxMsg.mode = 3;

    mcuPub.publish(dxMsg);
}

void init_ready2(Info_kinematics KM)
{
    serial_mcu::Motor_msg dxMsg;
    int i;
    //cout << "dyna 2" << endl;
    for(i = 0; i < KM.KM_flag; i++)
    {
        dxMsg.id.push_back(KM.Using_Motor_Id[i]);
        int pos = g_DXL_ID_position[KM.Using_Motor_Id[i]];
        if(g_Motor[KM.Using_Motor_Id[i]] == 2)
        {
            pos <<= 2;
        }
        dxMsg.position.push_back(pos);
        if(speed[i] == 0)
            dxMsg.speed.push_back(50);
        else
            dxMsg.speed.push_back(speed[i] * 50);

    }
    dxMsg.length = dxMsg.id.size();
    dxMsg.mode = 3;

    mcuPub.publish(dxMsg);
}

void init_ready3(double ms_time, bool starting_flag)
{
    //cout << "dyna 3" << endl;
    double speed_Min = 500;
    for(int i = 0; i < 23; i++)
    {
        double pos = positionToAngle(Only_Plus2(g_DXL_ID_Past_position[i] - g_DXL_ID_position[i]));
        //cout << i << " " << positionToAngle(Only_Plus2(g_DXL_ID_Past_position[i] - g_DXL_ID_position[i])) << endl;
        //cout << "Past = " << g_DXL_ID_Past_position[i] << "   now = " << g_DXL_ID_position[i] << endl;
        speed[i] = pos / (ms_time * ((360 * 0.114) / (60 * 1000)));
        //cout << "speed[" << i << "] = " << speed[i] << endl;

        if(speed_Min > speed[i] && speed[i] != 0)
            speed_Min = speed[i];
    }

}

