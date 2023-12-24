
#ifndef EC_HW_PLATFORM_MY_MATH_H
#define EC_HW_PLATFORM_MY_MATH_H

#ifdef __cplusplus
extern "C" {
#endif
#define RADpS2RPM(num)(num / (3.1415926f/30.0f))
#define MY_PI 3.14159265358979323846
int sign(int val);
float Limit(float val, float min, float max);
float LoopLimit(float val, float min, float max);
float DeadBand(float val, float min, float max);

float Deg2Rad(float degree);
float Rad2Deg(float radian);
float Encoder2Degree(float ecd, float ecd_range);

float rpm2dps(float rpm);
float dps2rpm(float dps);
float rpm2radps(float rpm);
float radps2rpm(float radps);
float dps2radps(float dps);
float radps2dps(float radps);

#ifdef __cplusplus
}
#endif

#endif //EC_HW_PLATFORM_MY_MATH_H
