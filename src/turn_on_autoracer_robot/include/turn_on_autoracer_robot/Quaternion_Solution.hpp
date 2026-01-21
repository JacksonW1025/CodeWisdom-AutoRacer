#ifndef __QUATERNION_SOLUTION_H_
#define __QUATERNION_SOLUTION_H_

#include <sensor_msgs/msg/imu.hpp>

extern sensor_msgs::msg::Imu Mpu6050;

float InvSqrt(float number);
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az);

#endif
