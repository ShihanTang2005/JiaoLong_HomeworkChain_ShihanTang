//
// Created by ytz20 on 2023/11/18.
//

#ifndef PROJECT_PID_HPP
#define PROJECT_PID_HPP

class PID
{
public:
    PID();
    PID(float kp, float ki, float kd, float i_max, float out_max);

    float Calculate(float ref, float fdb);//目标，当前

public:
    float kp_, ki_, kd_;
    float i_max_, out_max_;//积分限幅,输出限幅
    float output_;

protected:
    float ref_, fdb_;//目标，当前
    float err_, err_sum_, last_err_;
    float pout_, iout_, dout_;
};

#endif //PROJECT_PID_HPP
