//
// Created by TSH-STUDIO 唐诗涵 on 2023/12/20.
//
#include "../inc/remotecontrol.h"
#include "../inc/motor.h"
#include "../inc/callback.h"
#include "../Inc/my_math.h"

PID spid=PID(20,10,0,1000,8);
PID ppid=PID();

Motor motor1=Motor(Motor::M2006,36,Motor::SPEED,ppid,spid);

extern RC_Ctl_t RC_CtrlData;

void MainControlLoop() {
    if(RC_CtrlData.rc.s1!=RC_SW_MID)
        motor1.Reset();
    else
        motor1.target_speed_=radps2rpm(RC_CtrlData.rc.ch0);
    motor1.Handle();
    MotorControlCANTx();

}
