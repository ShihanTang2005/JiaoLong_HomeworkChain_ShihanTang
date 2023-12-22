//
// Created by TSH-STUDIO 唐诗涵 on 2023/12/20.
//
#include "../inc/remotecontrol.h"
#include "../inc/motor.h"
#include "../inc/callback.h"
#include "../inc/my_math.h"
#include "../inc/maincontrol.h"

PID spid=PID(1,0.01,1,1000,8);
PID ppid=PID();

//Motor motor1=Motor(Motor::M2006,36,Motor::SPEED,ppid,spid);
Motor* can1_motor_[11];
Motor* can2_motor_[11];

//copied from 2023 old frame, delete kf filter
Motor GMY(Motor::GM6020, 1, Motor::POSITION_SPEED,    // type, ratio, method
          PID(20, 1.0, 6, 0.5, 480),             // ppid
          PID(500, 0, 0, 0, 30000),8);            // spid
Motor GMP(Motor::GM6020, 1, Motor::POSITION_SPEED,    // type, ratio, method
          PID(18, 1.5, 16, 3, 480),              // ppid
          PID(400, 0, 0, 0,30000),8);


extern RC_Ctl_t RC_CtrlData;

void MainControlLoop() {
    if(RC_CtrlData.rc.s1!=RC_SW_MID) {
        GMY.Reset();
        GMP.Reset();
    }
    else {
        GMY.target_angle_ = Rad2Deg(RC_CtrlData.rc.ch0);
        GMP.target_angle_ = Rad2Deg(RC_CtrlData.rc.ch2);
    }
    GMP.Handle();
    GMY.Handle();
    MotorControlCANTx(&GMP,0x1FF,&hcan1);
    MotorControlCANTx(&GMY,0x1FF,&hcan2);

}

void ControlPlatform(){

}
