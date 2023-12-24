//
// Created by TSH-STUDIO 唐诗涵 on 2023/12/20.
//
#include "../inc/remotecontrol.h"
#include "../inc/motor.h"
#include "../inc/callback.h"
#include "../inc/my_math.h"
#include "../inc/maincontrol.h"

#include "../inc/module_switch.h"

//Motor motor1=Motor(Motor::M2006,36,Motor::SPEED,ppid,spid);
Motor* can1_motor_[11];
Motor* can2_motor_[11];

//copied from 2023 old frame, delete kf filter
Motor GMY(Motor::GM6020, 1, Motor::POSITION_SPEED,    // type, ratio, method
          PID(40, 10, 20, 0.5, 480),             // ppid
          PID(40, 30, 20, 0, 30000),8);            // spid
Motor GMP(Motor::GM6020, 1, Motor::POSITION_SPEED,    // type, ratio, method
          PID(6, 10, 16, 3, 480),              // ppid
          PID(40, 8, 10, 0,30000),8);


extern RC_Ctl_t RC_CtrlData;

void MainControlLoop() {
#ifdef GIMBAL
    if(RC_CtrlData.rc.s1!=RC_SW_MID) {
        //GMY.Reset();
        GMP.Reset();
    }
    else {
        //GMY.target_angle_ = Rad2Deg((1.0f+RC_CtrlData.rc.ch0)*MY_PI);
        GMP.target_angle_ = Rad2Deg((1.1f+(-RC_CtrlData.rc.ch3+1.0f)/6.0f)*MY_PI);
				GMP.target_angle_ = Limit(GMP.target_angle_,212.0f,270.0f);
		}
    GMP.Handle();
    //GMY.Handle();
    MotorControlCANTx(&GMP,0x1ff,&hcan1);
    //MotorControlCANTx(&GMY,0x1ff,&hcan2);
#else
    if(RC_CtrlData.rc.s1!=RC_SW_MID) {
        GMY.Reset();
        //GMP.Reset();
    }
    else {
        GMY.target_angle_ = Rad2Deg(RC_CtrlData.rc.ch2/15.0f*MY_PI);
        //GMP.target_angle_ = Rad2Deg((1.0f+RC_CtrlData.rc.ch2)*MY_PI);
		}
    //GMP.Handle();
    GMY.Handle();
    //MotorControlCANTx(&GMP,0x1ff,&hcan1);
    MotorControlCANTx(&GMY,0x1ff,&hcan2);
#endif

}

