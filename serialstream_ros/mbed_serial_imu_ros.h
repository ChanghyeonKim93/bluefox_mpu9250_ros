#ifndef _IMU_STREAM_H_
#define _IMU_STREAM_H_

#include <iostream>
#include <string>
#include <ros/ros.h>

#include "mbed_serial_imu_ros/serial_comm_linux.h"

#include "common/union_struct.h"

class ImuStream{
public:
    ImuStream(ros::NodeHandle& nh)
    : nh_(nh){
        // Get parameters from roslaunch
        this->getParameters();

        // initialize serial comm.
        serial_comm_ = std::make_shared<SerialCommunicatorLinux>(portname_, baud_rate_);

        // run
        this->run();
    };

private:
    void getParameters(){
        portname_ = "/dev/ttyACM0";
        baud_rate_ = 460800;
    };

    void run(){
        std::cout << "IMU STREAM - run()\n";
        ros::Rate rate(1000);
        while(ros::ok()){
            if(receiveDataReady()) getMessage();

            ros::spinOnce();
            rate.sleep();
        }
    };

    bool receiveDataReady(){
        return serial_comm_->isReceiveReady();
    };

    int getMessage(){
        char buf[1024];
        int len = 0;

        len = serial_comm_->getMessage(buf);
        parseMessage(buf);

        return -1;
    };

    int parseMessage(char* buf){
        // Variables

        USHORT_UNION acc[3];
        USHORT_UNION gyro[3];
        USHORT_UNION mag[3];
        USHORT_UNION sec;
        UINT_UNION   usec;

        double t_now = 0;
        acc[0].bytes_[0] = buf[1];    acc[0].bytes_[1] = buf[0];
        acc[1].bytes_[0] = buf[3];    acc[1].bytes_[1] = buf[2];
        acc[2].bytes_[0] = buf[5];    acc[2].bytes_[1] = buf[4];

        gyro[0].bytes_[0] = buf[1+6]; gyro[0].bytes_[1] = buf[0+6];
        gyro[1].bytes_[0] = buf[3+6]; gyro[1].bytes_[1] = buf[2+6];
        gyro[2].bytes_[0] = buf[5+6]; gyro[2].bytes_[1] = buf[4+6];
        
        mag[0].bytes_[0] = buf[1+12]; mag[0].bytes_[1] = buf[0+12];
        mag[1].bytes_[0] = buf[3+12]; mag[1].bytes_[1] = buf[2+12];
        mag[2].bytes_[0] = buf[5+12]; mag[2].bytes_[1] = buf[4+12];
        
        sec.bytes_[0] = buf[18]; sec.bytes_[1] = buf[19];
        usec.bytes_[0] = buf[20]; usec.bytes_[1] = buf[21];
        usec.bytes_[2] = buf[22]; usec.bytes_[3] = buf[23];

        t_now   = ((double)sec.ushort_ + (double)usec.uint_/1000000.0);

        std::cout << std::setprecision(12) << " time: " <<  t_now << " [s] / "  << (short)acc[0].ushort_ <<"," << (short)acc[1].ushort_ << "," << (short)acc[2].ushort_ 
        << " / " << (short)gyro[0].ushort_ << "," << (short)gyro[1].ushort_ << "," << (short)gyro[2].ushort_
        << " / " << (short)mag[0].ushort_ << "," << (short)mag[1].ushort_ << "," << (short)mag[2].ushort_ << std::endl;

        return -1;
    };

private:
    std::shared_ptr<SerialCommunicatorLinux> serial_comm_;
    std::string portname_;
    int baud_rate_;

// ROS
private:
    ros::NodeHandle nh_;

};
#endif