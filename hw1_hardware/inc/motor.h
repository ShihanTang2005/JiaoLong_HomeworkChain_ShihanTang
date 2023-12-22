
#ifndef PROJECT_MOTOR_HPP
#define PROJECT_MOTOR_HPP

#include "PID.h"
#include "main.h"

class Motor {
public:
    enum Type {
        M3508,
        M2006,
        GM6020,
    };
    enum ControlMethod {
        POSITION_SPEED, // 位置，速度双环 PID 控制
        SPEED, // 速度单环 PID 控制
    };
    enum Mode {
        POWER_OFF, // 断电，控制量置零
        STOP, // 将目标速度置零，计算得出控制量，使得电机停转
        WORKING, // 电机正常工作
    };

public:
    Motor(const Type& type, const float& ratio, const ControlMethod& method,
          const PID& ppid, const PID& spid);//method==SPEED时，ppid=NULL,spid有效
    void Reset(); // 重置电机所有状态
    void Handle(); // 根据当前 mode_ 计算控制量
    void SetAngle(const float& target_angle); // 设置目标角度
    void SetSpeed(const float& target_speed); // 设置目标速度
    void canRxMsg(const uint8_t *rx_data);
public:
    struct MotorInfo {
        Type type;
        float ratio; // 减速比
        //M2006减速比为36:1
        float max_intensity;
    } info_;
    Mode mode_;
    ControlMethod method_;
    int16_t intensity_; // 控制量
    float target_angle_, target_speed_; // 期望角度、速度
    struct MotorData {
        float angle_cycle_count; //编码器累计转过的整圈数对应的输出端角度
        float angle; // 减速后的输出端角度 [度]
        float ecd_angle; // 编码器角度   [度]
        float last_ecd_angle;
        float rotate_speed; // 减速后的输出端转速 [rpm]
        float current; // 转矩电流
        float temp; // 电机温度
    } motor_data_;
    PID ppid_, spid_;
};

void MotorControlCANTx(void);
void MotorControlCANRx(CAN_HandleTypeDef *hcan,const CAN_RxHeaderTypeDef *rx_header,const uint8_t *rx_data);

#endif //PROJECT_MOTOR_HPP
