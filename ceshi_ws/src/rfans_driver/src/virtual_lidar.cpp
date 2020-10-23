#include <iostream>
#include <ros/ros.h>
#include "rfans_driver/Point.h"
#include <opencv2/core/core.hpp>

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "virtual_lidar");
    ros::NodeHandle nh;
    ros::Publisher lidar_msg_pub = nh.advertise<rfans_driver::Point>("/Point", 1000);
    ros::Rate loop_rate(5);
    double w_sigma = 0.15;
    cv::RNG rng;
    while (ros::ok())
    {
        rfans_driver::Point lidar_msg;
        //lidar_msg.a1 = 2.16 +1+ rng.gaussian(w_sigma);
        //lidar_msg.b1 = 1.47 + rng.gaussian(w_sigma);
        //lidar_msg.a2 = 2.37+1 + rng.gaussian(w_sigma);
        //lidar_msg.b2 = -1.66 + rng.gaussian(w_sigma);
        //lidar_msg.a3 = 7.28+1 + rng.gaussian(w_sigma);
        //lidar_msg.b3 = 1.55 + rng.gaussian(w_sigma);
        //lidar_msg.a4 = 7.89+1 + rng.gaussian(w_sigma);
        //lidar_msg.b4 = -1.67 + rng.gaussian(w_sigma);
        //lidar_msg.x1 = (lidar_msg.a1 + lidar_msg.a2) / 2;
        //lidar_msg.y1 = (lidar_msg.b1 + lidar_msg.b2) / 2;
        //lidar_msg.x2 = (lidar_msg.a3 + lidar_msg.a4) / 2;
        //lidar_msg.y2 = (lidar_msg.b3 + lidar_msg.b4) / 2;

        lidar_msg.a1 = 1.44;
        lidar_msg.b1 = 1.5;
        lidar_msg.a2 = 1.44;
        lidar_msg.b2 = -1.5;
        lidar_msg.a3 = 6.44;
        lidar_msg.b3 = 1.5;
        lidar_msg.a4 = 6.44;
        lidar_msg.b4 =-1.5;
        lidar_msg.x1 = (lidar_msg.a1 + lidar_msg.a2) / 2;
        lidar_msg.y1 = (lidar_msg.b1 + lidar_msg.b2) / 2;
        lidar_msg.x2 = (lidar_msg.a3 + lidar_msg.a4) / 2;
        lidar_msg.y2 = (lidar_msg.b3 + lidar_msg.b4) / 2;
        //lidar_msg.x2 = 1000;
        //lidar_msg.y2 = 1000;
        lidar_msg_pub.publish(lidar_msg);
        loop_rate.sleep();
        /* code for loop body */
    }
    
    return 0;
}