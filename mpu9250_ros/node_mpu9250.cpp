#include <iostream>
#include <string>

#include <ros/ros.h>
#include "mpu9250_ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpu9250_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("mpu9250_node - starts.");

	try{
        MPU9250ROS mpu9250_ros(nh);
	}
	catch (std::exception& e){
        ROS_ERROR(e.what());
	}

    ROS_INFO_STREAM("mpu9250_node - terminated.");
	return 0;
}