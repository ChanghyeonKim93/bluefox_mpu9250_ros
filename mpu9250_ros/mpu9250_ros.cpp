#include "mpu9250_ros.h"

MPU9250ROS::MPU9250ROS(ros::NodeHandle& nh) 
: nh_(nh) 
{
    imu_data_.acc_scale  = 8.0/32768.0 * 9.81; // m/s2
    imu_data_.gyro_scale = 1000.0/32768.0/(180.0)*M_PI; // rad/s
    imu_data_.mag_scale  = (10.0*4219.0)/32760.0; // milliGauss
    
    // Subscriber
    sub_serial_ = nh.subscribe<std_msgs::UInt8MultiArray>("/serial/from_nucleo",1, &MPU9250ROS::callbackSerial, this);

    // publisher
    pub_imu_ = nh.advertise<sensor_msgs::Imu>("/mpu9250/imu",1);
    pub_mag_ = nh.advertise<sensor_msgs::MagneticField>("/mpu9250/mag",1);
    
    // Run node.
    this->run();
};

void MPU9250ROS::run(){
    ros::Rate rate(20000); // 20,000 Hz loop
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
};

void MPU9250ROS::callbackSerial(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{   
    if(msg->data.size() == 25) {
        SHORT_UNION val;
        val.bytes_[0] = msg->data[0];
        val.bytes_[1] = msg->data[1];
        imu_data_.acc[0] = (double)val.short_ * imu_data_.acc_scale;

        val.bytes_[0] = msg->data[2];
        val.bytes_[1] = msg->data[3];
        imu_data_.acc[1] = (double)val.short_ * imu_data_.acc_scale;

        val.bytes_[0] = msg->data[4];
        val.bytes_[1] = msg->data[5];
        imu_data_.acc[2] = (double)val.short_ * imu_data_.acc_scale;


        val.bytes_[0] = msg->data[6];
        val.bytes_[1] = msg->data[7];
        imu_data_.gyro[0] = (double)val.short_ * imu_data_.gyro_scale;

        val.bytes_[0] = msg->data[8];
        val.bytes_[1] = msg->data[9];
        imu_data_.gyro[1] = (double)val.short_ * imu_data_.gyro_scale;

        val.bytes_[0] = msg->data[10];
        val.bytes_[1] = msg->data[11];
        imu_data_.gyro[2] = (double)val.short_ * imu_data_.gyro_scale;


        val.bytes_[0] = msg->data[12];
        val.bytes_[1] = msg->data[13];
        imu_data_.mag[0] = (double)val.short_ * imu_data_.mag_scale;

        val.bytes_[0] = msg->data[14];
        val.bytes_[1] = msg->data[15];
        imu_data_.mag[1] = (double)val.short_ * imu_data_.mag_scale;

        val.bytes_[0] = msg->data[16];
        val.bytes_[1] = msg->data[17];
        imu_data_.mag[2] = (double)val.short_ * imu_data_.mag_scale;

        USHORT_UNION sec;
        UINT_UNION   usec;
        sec.bytes_[0]  = msg->data[18];  sec.bytes_[1] = msg->data[19];
        usec.bytes_[0] = msg->data[20]; usec.bytes_[1] = msg->data[21];
        usec.bytes_[2] = msg->data[22]; usec.bytes_[3] = msg->data[23];

        uint8_t cam_trigger_state = msg->data[24];
        imu_data_.time = ((double)sec.ushort_ + (double)usec.uint_ / 1000000.0);
        
        // std::cout << "time : " << time_ << ", trigger: ";
        // std::cout << (cam_trigger_state == (uint8_t)CAMERA_TRIGGER_HIGH ? "triggered" : "not triggered") << std::endl;
        
        // Fill IMU data
        sensor_msgs::Imu msg_imu;
        msg_imu.header.stamp = ros::Time::now();

        msg_imu.angular_velocity.x    = imu_data_.gyro[0];
        msg_imu.angular_velocity.y    = imu_data_.gyro[1];
        msg_imu.angular_velocity.z    = imu_data_.gyro[2];
        
        msg_imu.linear_acceleration.x = imu_data_.acc[0];
        msg_imu.linear_acceleration.y = imu_data_.acc[1];
        msg_imu.linear_acceleration.z = imu_data_.acc[2];

        sensor_msgs::MagneticField msg_mag;
        msg_mag.header.stamp = msg_imu.header.stamp;
        
        msg_mag.magnetic_field.x = imu_data_.mag[0];
        msg_mag.magnetic_field.y = imu_data_.mag[1];
        msg_mag.magnetic_field.z = imu_data_.mag[2];

        pub_imu_.publish(msg_imu);
    }
};  