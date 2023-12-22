//
// Created by TSH-STUDIO 唐诗涵 on 2023/12/20.
//
#include "../inc/remotecontrol.h"
#include "../inc/motor.h"
#include "../inc/callback.h"
#include "../Inc/my_math.h"

PID spid=PID(1,0.01,1,1000,8);
PID ppid=PID();

Motor motor1=Motor(Motor::M2006,36,Motor::SPEED,ppid,spid);

//copied from 2023 old frame, delete kf filter
Motor GMY(Motor::GM6020, 1, Motor::POSITION_SPEED,    // type, ratio, method
          PID(20, 1.0, 6, 0.5, 480),             // ppid
          PID(500, 0, 0, 0, 30000));            // spid
Motor GMP(Motor::GM6020, 1, Motor::POSITION_SPEED,    // type, ratio, method
          PID(18, 1.5, 16, 3, 480),              // ppid
          PID(400, 0, 0, 0,30000));


extern RC_Ctl_t RC_CtrlData;

void MainControlLoop() {
    if(RC_CtrlData.rc.s1!=RC_SW_MID)
        motor1.Reset();
    else
        motor1.target_speed_=radps2rpm(RC_CtrlData.rc.ch0);
    motor1.Handle();
    MotorControlCANTx();

}

void ControlPlatform(){

}
