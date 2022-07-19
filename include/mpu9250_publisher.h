#ifndef _MPU9250_PUBLISHER_H_
#define _MPU9250_PUBLISHER_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/Imu.h>

#include "union_struct.h"

class MPU9250Publisher{
private:

    double time_;
    double acc_[3];
    double gyro_[3];
    double mag_[3];

    double acc_scale_;
    double gyro_scale_;
    double mag_scale_;


    ros::NodeHandle nh_;
    ros::Subscriber sub_serial_;
    ros::Publisher pub_imu_;

private:
    void callbackSerial(const std_msgs::Int8MultiArray::ConstPtr& msg){
        // ROS_INFO_STREAM("Data recv: " << msg->data.size() );
        
        if(msg->data.size() == 18){
            SHORT_UNION val;
            val.bytes_[0] = msg->data[0];
            val.bytes_[1] = msg->data[1];
            acc_[0] = (double)val.short_ * acc_scale_;

            val.bytes_[0] = msg->data[2];
            val.bytes_[1] = msg->data[3];
            acc_[1] = (double)val.short_ * acc_scale_;

            val.bytes_[0] = msg->data[4];
            val.bytes_[1] = msg->data[5];
            acc_[2] = (double)val.short_ * acc_scale_;


            val.bytes_[0] = msg->data[6];
            val.bytes_[1] = msg->data[7];
            gyro_[0] = (double)val.short_ * gyro_scale_;

            val.bytes_[0] = msg->data[8];
            val.bytes_[1] = msg->data[9];
            gyro_[1] = (double)val.short_ * gyro_scale_;

            val.bytes_[0] = msg->data[10];
            val.bytes_[1] = msg->data[11];
            gyro_[2] = (double)val.short_ * gyro_scale_;


            val.bytes_[0] = msg->data[12];
            val.bytes_[1] = msg->data[13];
            mag_[0] = (double)val.short_ * mag_scale_;

            val.bytes_[0] = msg->data[14];
            val.bytes_[1] = msg->data[15];
            mag_[1] = (double)val.short_ * mag_scale_;

            val.bytes_[0] = msg->data[16];
            val.bytes_[1] = msg->data[17];
            mag_[2] = (double)val.short_ * mag_scale_;

            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time::now();

            msg.angular_velocity.x = gyro_[0];
            msg.angular_velocity.y = gyro_[1];
            msg.angular_velocity.z = gyro_[2];
            
            msg.linear_acceleration.x = acc_[0];
            msg.linear_acceleration.y = acc_[1];
            msg.linear_acceleration.z = acc_[2];

            pub_imu_.publish(msg);
        }
    };  

    void run(){
        ros::Rate rate(4000);
        while(ros::ok()){
            ros::spinOnce();
            rate.sleep();
        }
    }

public:
    MPU9250Publisher(ros::NodeHandle& nh) 
    : nh_(nh) 
    {
        acc_scale_ = 8.0/32768.0 * 9.81; // m/s2
        gyro_scale_ = 1000.0/32768.0/(180.0)*M_PI; // rad/s
        mag_scale_   = 10.0*4219.0/32760.0; // milliGauss
        // Subscriber
        sub_serial_ = nh.subscribe<std_msgs::Int8MultiArray>("/serial/from_nucleo",1, &MPU9250Publisher::callbackSerial, this);

        // publisher
        pub_imu_ = nh.advertise<sensor_msgs::Imu>("/mpu9250/imu",1);
        
        this->run();
    };


};
#endif