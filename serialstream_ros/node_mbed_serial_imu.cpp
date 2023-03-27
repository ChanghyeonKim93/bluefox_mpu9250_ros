#include <iostream>
#include <string>

#include <ros/ros.h>
#include "mbed_serial_imu_ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mbed_serial_imu_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("mbed_serial_imu_node - starts.");

	try{
        MbedSerialImuROS mpu9250_pub(nh);
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("mbed_serial_imu_node - terminated.");
	return 0;
}