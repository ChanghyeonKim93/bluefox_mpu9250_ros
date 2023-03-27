#ifndef _MPU9250_ROS_H_
#define _MPU9250_ROS_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "union_struct.h"

#define CAMERA_TRIGGER_LOW  0b01010101
#define CAMERA_TRIGGER_HIGH 0b10101010

struct IMUData{
    double time;
    
    double acc[3];
    double gyro[3];
    double mag[3];

    double acc_scale;
    double gyro_scale;
    double mag_scale;
};

class MPU9250ROS
{
// Constructor
public:
    MPU9250ROS(ros::NodeHandle& nh) ;
    
// Private methods
private:
    void run();
    void callbackSerial(const std_msgs::UInt8MultiArray::ConstPtr& msg);

// IMU time, 3-D acc., 3-D gyro., 3-D mag.
private:
    IMUData imu_data_;

// ROS related
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_serial_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_mag_;

};
#endif