
#include "../inc/motor.h"





#include "../inc/motor.h"
#include "math.h"
#include "../inc/my_math.h"
#include "../inc/maincontrol.h"


void motor_init(){
    can1_motor_[0]=&GMP;
    can2_motor_[0]=&GMY;
}


Motor::Motor(const Type& type, const float& ratio, const ControlMethod& method,
             const PID& ppid, const PID& spid,uint16_t id)
{
    id_=id;
    info_.type=type;
    info_.ratio=ratio;
    mode_=Motor::WORKING;
    method_=method;
    intensity_=0;
    target_angle_=0;
    target_speed_=0;
    motor_data_.angle_cycle_count=0.0;
    motor_data_.angle=0.0;
    motor_data_.ecd_angle=0.0;
    motor_data_.last_ecd_angle=0.0;
    motor_data_.rotate_speed=0.0;
    motor_data_.current=0.0;
    motor_data_.temp=0.0;
    if(method_==Motor::POSITION_SPEED)
        ppid_=PID(ppid.kp_,(info_.ratio>0)?ppid.ki_:(-ppid.ki_),(info_.ratio>0)?ppid.kd_:(-ppid.kd_),ppid.i_max_,ppid.out_max_);
    spid_=PID(spid.kp_,(info_.ratio>0)?spid.ki_:(-spid.ki_),(info_.ratio>0)?spid.kd_:(-spid.kd_),spid.i_max_,spid.out_max_);
    if (info_.type == Motor::M3508) {
        info_.max_intensity = 16384;
    } else if (info_.type == Motor::M2006) {
        info_.max_intensity = 10000;
    } else if (info_.type == Motor::GM6020) {
        info_.max_intensity = 30000;
    } //set max_intensity


}
void Motor::Reset() // 重置电机所有状态
{
    mode_=Motor::WORKING;
    intensity_=0;
    target_angle_=0;
    target_speed_=0;
    motor_data_.angle_cycle_count=0.0;
    motor_data_.angle=0.0;
    motor_data_.ecd_angle=0.0;
    motor_data_.last_ecd_angle=0.0;
    motor_data_.rotate_speed=0.0;
    motor_data_.current=0.0;
    motor_data_.temp=0.0;
    if(method_==Motor::POSITION_SPEED)
        ppid_=PID(ppid_.kp_,(info_.ratio>0)?ppid_.ki_:(-ppid_.ki_),(info_.ratio>0)?ppid_.kd_:(-ppid_.kd_),ppid_.i_max_,ppid_.out_max_);
    spid_=PID(spid_.kp_,(info_.ratio>0)?spid_.ki_:(-spid_.ki_),(info_.ratio>0)?spid_.kd_:(-spid_.kd_),spid_.i_max_,spid_.out_max_);
}
uint16_t k1=1000;
void Motor::Handle() // 根据当前 mode_ 计算控制量
{
    if(mode_==Motor::POWER_OFF)// 断电，控制量置零
    {
        intensity_=0;
        return;
    }
    if(mode_==Motor::WORKING)// 电机正常工作
    {
        motor_data_.angle = motor_data_.ecd_angle / info_.ratio; //减速比为编码器角度/输出角度
        if (method_ == Motor::POSITION_SPEED) //双环控制，计算位置PID
            target_speed_ = ppid_.Calculate(target_angle_, motor_data_.angle);
    }
    else if(mode_==Motor::STOP)// 将目标速度置零，计算得出控制量，使得电机停转
    {
        target_speed_=0.0;
    }
    if(info_.type==Motor::M3508) //C620电调
        intensity_=(int16_t)(spid_.Calculate(target_speed_,motor_data_.rotate_speed)/10.0f*10000);
        //电调接受控制量-10000~10000，对应电流-10A~10A
    else if(info_.type==Motor::M2006) //C610电调
        intensity_=(int16_t)(spid_.Calculate(target_speed_,motor_data_.rotate_speed)/20*16384);
        //电调接受控制量-16384~16384，对应电流-20A~20A
    else if(info_.type==Motor::GM6020)
        intensity_=(int16_t)(spid_.Calculate(target_speed_,motor_data_.rotate_speed)/24*30000);
    //电调接受控制量-30000~30000，对应电压-24V~24V

}
void Motor::SetAngle(const float& target_angle) // 设置目标角度
{
    target_angle_=target_angle;
}
void Motor::SetSpeed(const float& target_speed) // 设置目标速度
{
    target_speed_=target_speed;
}



//extern Motor motor1;



extern Motor GMY;
extern Motor GMP;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


uint8_t motor_Tx_message[8]={0};
static CAN_TxHeaderTypeDef motor_Tx_header;
static uint32_t TxMailbox0;

void MotorControlCANTx(Motor* motor1,uint32_t stdid,CAN_HandleTypeDef *hcan)
{
    motor1->canTxMsg();
    motor_Tx_header.StdId=stdid;  //标识符
    motor_Tx_header.ExtId=0;
    motor_Tx_header.IDE=CAN_ID_STD;
    motor_Tx_header.RTR=CAN_RTR_DATA;
    motor_Tx_header.DLC=8;
    motor_Tx_header.TransmitGlobalTime=DISABLE;
    HAL_CAN_AddTxMessage(hcan, &motor_Tx_header, motor_Tx_message, &TxMailbox0);

}

void Motor::canTxMsg()
{
    uint16_t motor_index=id_;
    uint16_t intense = intensity_;
    motor_Tx_message[id_*2-2] = (uint8_t) (intense >> 8);//电流值的高8位
    motor_Tx_message[id_*2-1] = (uint8_t) (intense & 0xFF); //电流值的低八位

}

void MotorControlCANRx(CAN_HandleTypeDef *hcan,const CAN_RxHeaderTypeDef *rx_header,const uint8_t *rx_data)
{
    if(rx_header->StdId < 0x201 || rx_header->StdId > 0x20B) //不是DJI电机数据包
        return;
    uint16_t motor_index=rx_header->StdId - 0x200;
    if (hcan == &hcan1){
        switch(motor_index){
            case 1:
                motor1.canRxMsg(rx_data);
                break;
            case 8:
                GMP.canRxMsg(rx_data);  //can1 id 8
                break;
            default:
                break;
        }
    }else if(hcan == &hcan2){
        switch(motor_index){
            case 8:
                GMY.canRxMsg(rx_data);  //can2 id8
                break;
            default:
                break;
        }
    }


}

void Motor::canRxMsg(const uint8_t *rx_data) {
    //M3508最大空载转速为589rpm，在一个CAN周期中最多转动589rpm*1ms=3.534度
    //M2006最大空载转速为777rpm，在一个CAN周期中最多转动777rpm*1ms=4.662度
    //GM6020最大空载转速为320rpm，在一个CAN周期中最多转动320rpm*1ms=1.92度
    Motor::motor_data_.last_ecd_angle=Motor::motor_data_.ecd_angle;
    Motor::motor_data_.ecd_angle=Encoder2Degree((float)((uint16_t)rx_data[0]<<8|(uint16_t)rx_data[1]),8192);
    if(Motor::motor_data_.ecd_angle-Motor::motor_data_.last_ecd_angle>180) //0跳到360
    {
        Motor::motor_data_.angle_cycle_count -= 360 / Motor::info_.ratio;
        if(Motor::motor_data_.angle_cycle_count<0)
            Motor::motor_data_.angle_cycle_count+=360; //输出端也有360度的周期截断
    }
    else if(Motor::motor_data_.last_ecd_angle-Motor::motor_data_.ecd_angle>180) //360跳到0
    {
        Motor::motor_data_.angle_cycle_count += 360 / Motor::info_.ratio;
        if(Motor::motor_data_.angle_cycle_count>360)
            Motor::motor_data_.angle_cycle_count-=360; //输出端也有360度的周期截断
    }
    Motor::motor_data_.angle=Motor::motor_data_.ecd_angle/Motor::info_.ratio+Motor::motor_data_.angle_cycle_count;
    if(Motor::motor_data_.angle>360)
        Motor::motor_data_.angle-=360; //输出端也有360度的周期截断
    Motor::motor_data_.rotate_speed=(float)((int16_t)((uint16_t)rx_data[2]<<8|(uint16_t)rx_data[3]))/Motor::info_.ratio; //除以减速比
    Motor::motor_data_.temp=(float)rx_data[6];
}

