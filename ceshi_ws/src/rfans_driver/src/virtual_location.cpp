#include <iostream>
#include <ros/ros.h>
#include "rfans_driver/gps_data.h"
#include <opencv2/core/core.hpp>

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "virtual_location");
    ros::NodeHandle nh;
    ros::Publisher lidar_msg_pub = nh.advertise<rfans_driver::gps_data>("/read", 1000);
    ros::Rate loop_rate(5);
    double w_sigma1 = 0.0000003;
    double w_sigma2 = 0.07;
    cv::RNG rng;
    while (ros::ok())
    {
        rfans_driver::gps_data gps_data_msg;
       //gps_data_msg.latitude=31.000000+rng.gaussian(w_sigma1);
        //gps_data_msg.longitude=117.000000+rng.gaussian(w_sigma1);
       //gps_data_msg.yaw=0+rng.gaussian(w_sigma2);

        gps_data_msg.latitude=31.00000;
        gps_data_msg.longitude=117.25000;
        gps_data_msg.yaw=0;
        lidar_msg_pub.publish(gps_data_msg);
        loop_rate.sleep();
        /* code for loop body */
    }
    
    return 0;
}