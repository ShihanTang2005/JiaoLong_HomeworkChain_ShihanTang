//
// Created by ytz20 on 2023/11/18.
//

#include "../inc/my_math.h"

/**
 * @brief 限幅：输入小于下限，则输出下限。若输入大于上限，则输出上限
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 限幅后的数值
 */
float Limit(float val, float min, float max)
{
    if(val>=max)
        return max;
    if(val<=min)
        return min;
    return val;
}

/**
 * @brief 循环限幅：将最大值减最小值的差视为一个周期，加上或减去若干个周期，使得输出在下限与上限之间
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 循环限幅后的数值
 */
float LoopLimit(float val, float min, float max)
{
    float range=max-min;
    while(val>=max)
        val-=range;
    while(val<=min)
        val+=range;
    return val;
}

/**
 * @brief 死区：若输入在上限与下限之间，输出零，否则不进行操作
 * @param val 输入值
 * @param min 下限
 * @param max 上限
 * @return 死区操作后的数值
 */
float DeadBand(float val, float min, float max)
{
    if(val<max && val>min)
        return 0.0;
    return val;
}


/**
 * @brief 角度转弧度
 * @param degree 角度值
 * @return 弧度值
 */
float Deg2Rad(float degree)
{
    return degree/180*MY_PI;
}

/**
 * @brief 弧度转角度
 * @param radian 弧度值
 * @return 角度值
 */
float Rad2Deg(float radian)
{
    return radian/MY_PI*180;
}

/**
 * @brief 编码器角度转角度值
 * @param ecd 编码器角度
 * @param ecd_range 一圈对应的编码器角度，例如大疆电机为 8192
 * @return 角度值
 */
float Encoder2Degree(float ecd, float ecd_range)
{
    return ecd/ecd_range*360;
}


/**
 * @brief 圈每分钟 转 度每秒
 * @param rpm 圈每分钟
 * @return 度每秒
 */
float rpm2dps(float rpm)
{
    //return rpm/60*360;
    return rpm*60;
}

/**
 * @brief 度每秒 转 圈每分钟
 * @param dps 度每秒
 * @return 圈每分钟
 */
float dps2rpm(float dps)
{
    return dps/60;
}

/**
 * @brief 圈每分钟 转 弧度每秒
 * @param rpm 圈每分钟
 * @return 弧度每秒
 */
float rpm2radps(float rpm)
{
    //return rpm*(2*MY_PI)/60;
    return rpm*MY_PI/30;
}

/**
 * @brief 弧度每秒 转 圈每分钟
 * @param radps 弧度每秒
 * @return 圈每分钟
 */
float radps2rpm(float radps)
{
    return radps*30/MY_PI;
}

/**
 * @brief 度每秒 转 弧度每秒
 * @param dps 度每秒
 * @return 弧度每秒
 */
float dps2radps(float dps)
{
    return dps/180*MY_PI;
}

/**
 * @brief 弧度每秒 转 度每秒
 * @param radps 弧度每秒
 * @return 度每秒
 */
float radps2dps(float radps)
{
    return radps/MY_PI*180;
}