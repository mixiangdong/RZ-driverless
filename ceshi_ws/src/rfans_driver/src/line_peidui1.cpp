//修改数据类型，GPS坐标转化的DOUBLE	和 double64
//弄清楚GPS数据形式修改方向差的BUG
//修改头文件
#include "rfans_driver/Point.h"

#include "rfans_driver/gps_data.h"
#include "rfans_driver/null.h"
#include "rfans_driver/point_zx.h"
#include "rfans_driver/steering_wheel_angle.h"
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>
#include <std_msgs/String.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
//包含自定义msg产生的头文件
// ROS标准msg头文件
#include <serial/serial.h> //ROS已经内置了的串口包
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
using namespace std;

#define spoint struct point
#define lidar_center_dis_4 1.05
#define lidar_center_dis_2 0.75
#define PI 3.14159265359
#define steering_device_error 7
#define PID_I_time 5
#define PID_initial_time 60
#define loop_times 2

struct point {
  double x;
  double y;
  double yaw;
};

typedef struct {
  double setValue;
  double actualValue;
  double controlValue;
  double lastLastError;
  double lastError;
  double currentError;
  double SigmaError;
  double Kp, Ki, Kd;
} pid_t_;

spoint left1, left1_save, left2, right1, right2, middle1, middle2, middle2_,
    center, point[6], point_pair[3][2], temp_point, temp_left, temp_right;
double k0, k, b, lateralerror, theta, a_fa, lateral, dis_point_point, a[6],
    bb[6], steering_wheel_angle_save_1, steering_wheel_angle_save_2,
    left1_left1_save_dis;
rfans_driver::steering_wheel_angle steering_wheel_angle;
visualization_msgs::Marker line_list, line_list2, line_list3, line_list4,
    line_list5, line_list6, line_list7, line_list8;
int point_state, i1, i2, i3, i4, times_flag, num;
pid_t_ yaw_pid;          //航向角偏差
pid_t_ lateralError_pid; //横向偏差
double pid_yaw;
double pid_lateraError;
ros::Publisher marker_pub;
ros::Publisher control_pub;
serial::Serial ser;

int main(int argc, char **argv) {

  void PID_init_yaw(pid_t_ * pid);
  void PID_init_lateralError(pid_t_ * pid);
  double PID2_realize(pid_t_ * pid, double setValue, double currentValue);
  double PID1_realize(pid_t_ * pid, double setValue, double currentValue);
  int cross_multiply(spoint middle1, spoint middle2, spoint center);
  void distance(spoint p1, spoint p2, double *dis);
  void vector_multiply(double angle, spoint center, spoint location,
                       double *multiply);
  double LateralError(double t_yaw_start, double t_yaw_now, double dis2end);
  // void callback2(const rfans_driver::Point_disorder::ConstPtr &point_msg);
  // void callback2(const rfans_driver::Point::ConstPtr &point_msg);
  void callback2(const rfans_driver::point_zx::ConstPtr &point_msg);
  PID_init_yaw(&yaw_pid);
  PID_init_lateralError(&lateralError_pid);
  ros::init(argc, argv, "upper_control");
  ros::NodeHandle nh;
  marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  control_pub = nh.advertise<rfans_driver::steering_wheel_angle>(
      "steering_wheel_angle", 1000);
  ros::Subscriber sub2 = nh.subscribe("Point", 1, callback2);
  /*while (ros::ok()) {
    control_pub.publish(steering_wheel_angle);
    sleep(1); //根据前面的定义的loop_rate,设置1s的暂停
  }
  */
  // serial::Serial ser;
  /*
    try {
      //设置串口属性，并打开串口
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
    } catch (serial::IOException &e) {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if (ser.isOpen()) {
      ROS_INFO_STREAM("Serial Port initialized");
    } else {
      return -1;
    }
    */
  ros::spin();
  // ros::Duration(0.5).sleep(); //每次循环延迟0.5秒
  // loop_rate.sleep();

  // ros::spin();

  return 0;
}

void PID_init_yaw(pid_t_ *pid) {
  pid->setValue = 0.0;
  pid->actualValue = 0.0;
  pid->controlValue = 0.0;
  pid->lastLastError = 0.0;
  pid->lastError = 0.0;
  pid->currentError = 0.0;
  pid->SigmaError = 0.0;
  pid->Kp = 1 * 1.2 * 4;
  pid->Ki = 0;
  pid->Kd = 0.0;
}

//对横向误差的PID参数初始化
void PID_init_lateralError(pid_t_ *pid) {
  pid->setValue = 0.0;
  pid->actualValue = 0.0;
  pid->controlValue = 0.0;
  pid->lastLastError = 0.0;
  pid->lastError = 0.0;
  pid->currentError = 0.0;
  pid->SigmaError = 0.0;
  pid->Kp = 45 / 2 * 4;
  pid->Ki = 0;
  pid->Kd = 0.0;
}

void PID_init_yaw2(pid_t_ *pid) {
  pid->setValue = 0.0;
  pid->actualValue = 0.0;
  pid->controlValue = 0.0;
  pid->lastLastError = 0.0;
  pid->lastError = 0.0;
  pid->currentError = 0.0;
  pid->SigmaError = 0.0;
  // pid->Kp = 1 * 1.25 * 4 ;
  pid->Kp = 1 * 1.15 * 4;
  pid->Ki = 0;
  pid->Kd = 0.0;
}

//对横向误差的PID参数初始化
void PID_init_lateralError2(pid_t_ *pid) {
  pid->setValue = 0.0;
  pid->actualValue = 0.0;
  pid->controlValue = 0.0;
  pid->lastLastError = 0.0;
  pid->lastError = 0.0;
  pid->currentError = 0.0;
  pid->SigmaError = 0.0;
  // pid->Kp = 45 / 2 * 4 / 1;
  pid->Kp = 45 / 2 * 4 / 1.1;
  pid->Ki = 45 / 2 * 4 / PID_I_time / loop_times;
  // pid->Ki = 0;
  pid->Kd = 0.0;
}
//位置PID
double PID2_realize(pid_t_ *pid, double setValue, double currentValue) {
  pid->actualValue = currentValue;
  pid->setValue = setValue;
  pid->currentError =
      pid->setValue - pid->actualValue; //当前误差，即设置值与当前值的差
  pid->SigmaError += pid->currentError; //σ误差，为积分项
  if (fabs(pid->currentError) < 0.10)   //如果当前误差小于0.1
    pid->SigmaError = 0.0;
  pid->controlValue = (pid->Kp * pid->currentError + pid->Ki * pid->SigmaError +
                       pid->Kd * (pid->currentError - pid->lastError));

  /*ROS_INFO("\ncurrentError=%f  lastError=%f  control_value=%f p=%.3f  i=%.3f
     "
           " d=%.3f\n",
           pid->currentError, pid->lastError, pid->controlValue, pid->Kp,
           pid->Ki, pid->Kd);
*/
  pid->lastError = pid->currentError;
  pid->lastLastError = pid->lastError;

  return pid->controlValue;
}

//增量PID
double PID1_realize(pid_t_ *pid, double setValue, double currentValue) {
  pid->actualValue = currentValue;
  pid->setValue = setValue;
  pid->currentError = pid->setValue - pid->actualValue;
  pid->controlValue +=
      pid->Kp * (pid->currentError - pid->lastError) +
      pid->Ki * pid->currentError +
      pid->Kd * (pid->currentError - 2 * pid->lastError + pid->lastLastError);
  pid->lastError = pid->currentError;
  pid->lastLastError = pid->lastError;

  return pid->controlValue;
}

int cross_multiply(spoint middle1, spoint middle2, spoint center) {
  double x;
  x = (middle2.x - middle1.x) * (center.y - middle1.y) -
      (middle2.y - middle1.y) * (center.x - middle1.x);
  if (x > 0) //点在左侧
    return 1;
  else if (x = 0) //点在线上
    return 0;
  else
    return -1; //点在右侧
}

//求两点距离，前点指向后点。
void distance(spoint p1, spoint p2, double *dis) {
  *dis = sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

//求向量内积。函数体默认GPS正东为0度，形式可以为0-360，也可以为-180-+180。
void vector_multiply(double angle, spoint center, spoint location,
                     double *multiply) {
  double arc, x1, x2, y1, y2, cha_x, cha_y, length;
  arc = angle / 180 * PI;
  x1 = cos(arc);
  y1 = sin(arc);
  cha_x = center.x - location.x;
  cha_y = center.y - location.y;
  length = sqrt(cha_x * cha_x + cha_y * cha_y);
  x2 = cha_x / length;
  y2 = cha_y / length;
  *multiply = x1 * x2 + y1 * y2;
}

//求横向偏差，根据期望航向角（上一个锥桶或发车位置指向目标点的方向）、当前点指向目标点的方向以及当前点和目标点的距离计算
//为-时，车在计划路径的右侧，（有待讨论符号问题）
double LateralError(double t_yaw_start, double t_yaw_now, double dis2end) {
  return ((sin((t_yaw_start - t_yaw_now) * PI / 180)) * dis2end);
}

// void callback2(const rfans_driver::Point_disorder::ConstPtr &point_msg) {
void callback2(const rfans_driver::point_zx::ConstPtr &point_msg) {
  // sleep(0.5);
  ros::Time::init();
  ros::Rate loop_rate(loop_times);
  /*
  if(times_flag==PID_initial_time*loop_times)
  {
    PID_init_yaw2(&yaw_pid);
    PID_init_lateralError2(&lateralError_pid);
  }
  times_flag++;
  */
  // while (ros::ok()) {

  a[0] = point_msg->x1;
  a[1] = point_msg->x2;
  a[2] = point_msg->x3;
  a[3] = point_msg->x4;
  a[4] = point_msg->x5;
  a[5] = point_msg->x6;
  bb[0] = point_msg->y1;
  bb[1] = point_msg->y2;
  bb[2] = point_msg->y3;
  bb[3] = point_msg->y4;
  bb[4] = point_msg->y5;
  bb[5] = point_msg->y6;
  /*
  a[0]=point_msg->a1;
  a[1]=point_msg->a2;
  a[2]=point_msg->a3;
  a[3]=point_msg->a4;
  bb[0]=point_msg->b1;
  bb[1]=point_msg->b2;
  bb[2]=point_msg->b3;
  bb[3]=point_msg->b4;
  */
  i4 = 0;
  for (i2 = 0; i2 < 3; i2++) {
    for (i3 = 0; i3 < 2; i3++) {
      point_pair[i2][i3].x = 1000;
      point_pair[i2][i3].y = 1000;
    }
  }
  for (i1 = 0; i1 < 6; i1++) {
    point[i1].x = a[i1];
    point[i1].y = bb[i1];
  }
  for (i2 = 0; i2 < 6; i2++) {
    if (point[i2].x == 1000 || point[i2].y == 1000) {
      continue;
    }
    for (i3 = i2 + 1; i3 < 6; i3++) {
      if (point[i3].x == 1000 || point[i3].y == 1000) {
        continue;
      }
      distance(point[i2], point[i3], &dis_point_point);
      if (dis_point_point > 2.5 && dis_point_point < 3.5) {
        point_pair[i4][0] = point[i2];
        point_pair[i4][1] = point[i3];
        i4++;
        point[i2].x = 1000;
        point[i2].y = 1000;
        point[i3].x = 1000;
        point[i3].y = 1000;
        break;
      }
    }
  }
  cout << "有效桶对数: " << i4 << endl;
  //分清左右
  for (i1 = 0; i1 < i4; i1++) {
    if (point_pair[i1][0].y < point_pair[i1][1].y) {
      temp_point = point_pair[i1][1];
      point_pair[i1][1] = point_pair[i1][0];
      point_pair[i1][0] = temp_point;
    }
  }
  //分清前后,冒泡排序
  if (i4 > 1) {
    for (i2 = 0; i2 < i4 - 1; i2++) {
      for (i3 = 0; i3 < i4 - 1 - i2; i3++) {
        if (point_pair[i3][0].x > point_pair[i3 + 1][0].x) {
          temp_left = point_pair[i3 + 1][0];
          point_pair[i3 + 1][0] = point_pair[i3][0];
          point_pair[i3][0] = temp_left;
          temp_right = point_pair[i3 + 1][1];
          point_pair[i3 + 1][1] = point_pair[i3][1];
          point_pair[i3][1] = temp_right;
        }
      }
    }
  } else {
  }
  if (i4 > 1) {
    left1 = point_pair[0][0];
    left2 = point_pair[1][0];
    right1 = point_pair[0][1];
    right2 = point_pair[1][1];
    middle1.x = (left1.x + right1.x) / 2;
    middle2.x = (left2.x + right2.x) / 2;
    middle1.y = (left1.y + right1.y) / 2;
    middle2.y = (left2.y + right2.y) / 2;
  } else if (i4 == 1) {
    left1 = point_pair[0][0];
    right1 = point_pair[0][1];
    middle1.x = (left1.x + right1.x) / 2;
    middle1.y = (left1.y + right1.y) / 2;
    middle2.x = 1000;
    middle2.y = 1000;
  } else {
    middle1.x = 1000;
    middle1.y = 1000;
    middle2.x = 1000;
    middle2.y = 1000;
  }
  ROS_INFO("left2.x: %f,left2.y: %f,right2.x: %f,right2.y: %f\n", left2.x,
           left2.y, right2.x, right2.y);
  ROS_INFO("left1.x: %f,left1.y: %f,right1.x: %f,right1.y: %f\n", left1.x,
           left1.y, right1.x, right1.y);
  ROS_INFO("middle1.x: %f,middle1.y: %f,middle2.x: %f,middle2.y: %f,\n\n\n",
           middle1.x, middle1.y, middle2.x, middle2.y);

  if (left1.x != 1000 && left1.y != 1000 && left1_save.x != 1000 &&
      left1_save.y != 1000 && left1.x != 0 && left1.y != 0 &&
      left1_save.x != 0 && left1_save.y != 0) {
    distance(left1, left1_save, &left1_left1_save_dis);
    if (left1_left1_save_dis > 3&&left1.x>left1_save.x) {
      num++;
    }
  }
  left1_save = left1;
  ROS_INFO("num: %d\n", num);

  line_list5.header.frame_id = "pandar";
  line_list5.header.stamp = ros::Time::now();
  line_list5.ns = "lines5";
  line_list5.action = visualization_msgs::Marker::ADD;
  // 四元数变换:q=((x,y,z)sin(theta/2),cos(theta/2));
  line_list5.pose.orientation.w = 1.0;
  line_list5.pose.position.x = -1 * lidar_center_dis_4;
  line_list5.pose.position.y = 0;
  line_list5.pose.position.z = 0;
  line_list5.id = 5;
  line_list5.lifetime = ros::Duration();
  line_list5.type = visualization_msgs::Marker::ARROW;
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for
  // the line width
  line_list5.scale.x = 5;
  line_list5.scale.y = 0.1;
  line_list5.scale.z = 0.1;
  // Line list is red
  line_list5.color.r = 1.0;
  line_list5.color.a = 1.0;
  // Create the vertices for the points and lines
  /*geometry_msgs::Point p5;
   p5.x = -1 * lidar_center_dis_4;
   p5.y = 0;
   p5.z = 0;
   // The line list needs two points for each line
   line_list5.points.push_back(p5);
   p5.x = middle2.x;
   p5.y = middle2.y;
   p5.z = 0;
   line_list5.points.push_back(p5);*/
  marker_pub.publish(line_list5);

  // for (;;) {

  line_list.points.clear();
  line_list2.points.clear();
  line_list3.points.clear();
  line_list4.points.clear();
  line_list6.points.clear();
  // line_list7.points.clear();
  // line_list8.points.clear();
  if (i4 == 2) {
    center.x = -1 * lidar_center_dis_4;
    center.y = 0;
  } else {
    center.x = -1 * lidar_center_dis_2;
    center.y = 0;
  }
  /*
  center.x = -1 * lidar_center_dis_4;
  center.y = 0;
  */
  if (middle2.x == 1000 && middle2.y == 1000 && middle1.x != 1000 &&
      middle1.y != 1000) {
    if (left1.y - right1.y == 0) {
      steering_wheel_angle.data = 1000;
      point_state = 1;

    }

    else if (fabs(left1.x - right1.x) < 0.001) {
      steering_wheel_angle.data = 0;
      point_state = 1;

    } else {
      k0 = (left1.y - right1.y) / (left1.x - right1.x);
      k = -1 / k0;
      b = middle1.y - k * middle1.x;
      ROS_INFO("k: %.9f\n", k);
      ROS_INFO("b: %.9f\n", b);
      lateralerror = fabs(k * center.x + b - center.y) / sqrt(k * k + 1);
      middle2_.x = middle1.x + 5;
      middle2_.y = k * middle2_.x + b;
      theta = -1 * atan(k) / PI *
              180; //如果为-，应该逆时针打方向，PID输出+，反之亦反

      lateral = 1 * lateralerror *
                cross_multiply(middle1, middle2_,
                               center); //如果为-，应该逆时针打方向，反之亦反

      pid_yaw = PID2_realize(&yaw_pid, 0.0, theta);
      pid_lateraError = PID2_realize(&lateralError_pid, 0.0, lateral);
      steering_wheel_angle.data = -1 * (pid_yaw + pid_lateraError);
      point_state = 1;
    }

    line_list4.header.frame_id = "pandar";
    line_list4.header.stamp = ros::Time::now();
    line_list4.ns = "lines4";
    line_list4.action = visualization_msgs::Marker::MODIFY;
    line_list4.pose.orientation.w = 1.0;
    line_list4.id = 4;
    line_list4.lifetime = ros::Duration(0.5);
    line_list4.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for
    // the line width
    line_list4.scale.x = 0.1;
    // Line list is red
    line_list4.color.b = 1.0;
    line_list4.color.a = 1.0;
    // Create the vertices for the points and lines
    geometry_msgs::Point p4;
    p4.x = middle1.x;
    p4.y = middle1.y;
    p4.z = 0;
    // The line list needs two points for each line
    line_list4.points.push_back(p4);
    p4.x = middle2_.x;
    p4.y = middle2_.y;
    p4.z = 0;
    line_list4.points.push_back(p4);
    marker_pub.publish(line_list4);

    line_list7.header.frame_id = "pandar";
    line_list7.header.stamp = ros::Time::now();
    line_list7.ns = "lines7";
    line_list7.action = visualization_msgs::Marker::ADD;
    line_list7.pose.orientation.w = 1.0;
    line_list7.id = 7;
    line_list7.lifetime = ros::Duration(0.5);
    line_list7.type = visualization_msgs::Marker::SPHERE;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for
    // the line width
    line_list7.scale.x = 0.2;
    line_list7.scale.y = 0.2;
    line_list7.scale.z = 0.2;
    // Line list is red
    line_list7.color.r = 1.0;
    line_list7.color.a = 1.0;
    // Create the vertices for the points and lines
    /* geometry_msgs::Point p7;
     p7.x = right1.x;
     p7.y = right1.y;
     p7.z = 0;*/
    line_list7.pose.position.x = left1.x;
    line_list7.pose.position.y = left1.y;
    line_list7.pose.position.z = 0;
    // line_list7.points.push_back(p7);
    marker_pub.publish(line_list7);

    line_list8.header.frame_id = "pandar";
    line_list8.header.stamp = ros::Time::now();
    line_list8.ns = "lines8";
    line_list8.action = visualization_msgs::Marker::ADD;
    line_list8.pose.orientation.w = 1.0;
    line_list8.id = 8;
    line_list8.lifetime = ros::Duration(0.5);
    line_list8.type = visualization_msgs::Marker::SPHERE;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for
    // the line width
    line_list8.scale.x = 0.2;
    line_list8.scale.y = 0.2;
    line_list8.scale.z = 0.2;
    // Line list is red
    line_list8.color.b = 1.0;
    line_list8.color.a = 1.0;
    // Create the vertices for the points and lines
    /* geometry_msgs::Point p7;
     p7.x = right1.x;
     p7.y = right1.y;
     p7.z = 0;*/
    line_list8.pose.position.x = right1.x;
    line_list8.pose.position.y = right1.y;
    line_list8.pose.position.z = 0;
    // line_list7.points.push_back(p7);
    marker_pub.publish(line_list8);

  } else if (middle2.x != 1000 && middle2.y != 1000 && middle1.x != 1000 &&
             middle1.y != 1000) {

    line_list.header.frame_id = "pandar";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::MODIFY;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 1;
    line_list.lifetime = ros::Duration(0.5);
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for
    // the line width
    line_list.scale.x = 0.02;
    // Line list is red
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
    // Create the vertices for the points and lines
    geometry_msgs::Point p;
    p.x = left1.x;
    p.y = left1.y;
    p.z = 0;
    // The line list needs two points for each line
    line_list.points.push_back(p);
    p.x = left2.x;
    p.y = left2.y;
    p.z = 0;
    line_list.points.push_back(p);
    marker_pub.publish(line_list);

    line_list2.header.frame_id = "pandar";
    line_list2.header.stamp = ros::Time::now();
    line_list2.ns = "lines2";
    line_list2.action = visualization_msgs::Marker::MODIFY;
    line_list2.pose.orientation.w = 1.0;
    line_list2.id = 2;
    line_list2.lifetime = ros::Duration(0.5);
    line_list2.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for
    // the line width
    line_list2.scale.x = 0.02;
    // Line list is red
    line_list2.color.g = 1.0;
    line_list2.color.a = 1.0;
    // Create the vertices for the points and lines
    geometry_msgs::Point p2;
    p2.x = right1.x;
    p2.y = right1.y;
    p2.z = 0;
    // The line list needs two points for each line
    line_list2.points.push_back(p2);
    p2.x = right2.x;
    p2.y = right2.y;
    p2.z = 0;
    line_list2.points.push_back(p2);
    marker_pub.publish(line_list2);

    line_list3.header.frame_id = "pandar";
    line_list3.header.stamp = ros::Time::now();
    line_list3.ns = "lines3";
    line_list3.action = visualization_msgs::Marker::MODIFY;
    line_list3.pose.orientation.w = 1.0;
    line_list3.id = 3;
    line_list3.lifetime = ros::Duration(0.5);
    line_list3.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for
    // the line width
    line_list3.scale.x = 0.1;
    // Line list is red
    line_list3.color.r = 1.0;
    line_list3.color.a = 1.0;
    // Create the vertices for the points and lines
    geometry_msgs::Point p3;
    p3.x = middle1.x;
    p3.y = middle1.y;
    p3.z = 0;
    // The line list needs two points for each line
    line_list3.points.push_back(p3);
    p3.x = middle2.x;
    p3.y = middle2.y;
    p3.z = 0;
    line_list3.points.push_back(p3);
    marker_pub.publish(line_list3);

    k = (middle1.y - middle2.y) / (middle1.x - middle2.x);
    b = (middle2.x * middle1.y - middle1.x * middle2.y) /
        (middle2.x - middle1.x);
    ROS_INFO("k: %.9f\n", k);
    ROS_INFO("b: %.9f\n", b);
    lateralerror = fabs(k * center.x + b - center.y) / sqrt(k * k + 1);
    theta = -1 * atan(k) / PI * 180; //如果为-，应该逆时针打方向，反之亦反

    ROS_INFO("theta: %.9f\n", theta);

    lateral = 1 * lateralerror *
              cross_multiply(middle1, middle2,
                             center); //如果为-，应该逆时针打方向，反之亦反

    ROS_INFO("lateral: %.9f\n", lateral);

    pid_yaw = PID2_realize(&yaw_pid, 0.0, theta);
    pid_lateraError = PID2_realize(&lateralError_pid, 0.0, lateral);
    steering_wheel_angle.data = -1 * (pid_yaw + pid_lateraError);

    point_state = 2;
  } else {
    steering_wheel_angle.data = steering_wheel_angle.data;
    pid_yaw = 1000;
    pid_lateraError = 1000;
    point_state = 0;
  }
  /*
    if ((steering_wheel_angle.data - steering_wheel_angle_save_1) *
            (steering_wheel_angle_save_1 - steering_wheel_angle_save_2) >
        0) {
      steering_wheel_angle.data = steering_wheel_angle.data;
    } else if ((steering_wheel_angle.data - steering_wheel_angle_save_1) *
                   (steering_wheel_angle_save_1 - steering_wheel_angle_save_2) <
               0) {
      if (steering_wheel_angle.data - steering_wheel_angle_save_1 > 0) {
        steering_wheel_angle.data =
            steering_wheel_angle.data + steering_device_error;
      } else {
        steering_wheel_angle.data =
            steering_wheel_angle.data - steering_device_error;
      }
    }
    */
  if (steering_wheel_angle.data > 72) {
    cout << steering_wheel_angle.data << endl;
    steering_wheel_angle.data = 72;
  } else if (steering_wheel_angle.data < -1 * 72) {
    cout << steering_wheel_angle.data << endl;
    steering_wheel_angle.data = -1 * 72;
  } else {
    steering_wheel_angle.data = steering_wheel_angle.data;
  }

  // steering_wheel_angle_save_2 = steering_wheel_angle_save_1;
  // steering_wheel_angle_save_1 = steering_wheel_angle.data;
  if(num>=16)
  {
    steering_wheel_angle.data=360;
  }
  ROS_INFO("pid_yaw: %.9f\n", pid_yaw);
  ROS_INFO("pid_lateraError: %.9f\n", pid_lateraError);
  ROS_INFO("steering_wheel_angle.data: %.9f\n\n\n", steering_wheel_angle.data);
  ROS_INFO("point_state: %d\n", point_state);
  ROS_INFO("------------------\n");
  control_pub.publish(steering_wheel_angle);

  line_list6.header.frame_id = "pandar";
  line_list6.header.stamp = ros::Time::now();
  line_list6.ns = "lines6";
  line_list6.action = visualization_msgs::Marker::ADD;
  // 四元数变换:q=((x,y,z)sin(theta/2),cos(theta/2));
  line_list6.pose.orientation.z =
      sin(-1 * steering_wheel_angle.data / 180 * PI / 2);
  line_list6.pose.orientation.w =
      cos(-1 * steering_wheel_angle.data / 180 * PI / 2);
  line_list6.id = 6;
  line_list6.lifetime = ros::Duration();
  line_list6.type = visualization_msgs::Marker::ARROW;
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for
  // the line width
  line_list6.scale.x = 1.0;
  line_list6.scale.y = 0.1;
  line_list6.scale.z = 0.1;
  // Line list is red
  line_list6.color.g = 1.0;
  line_list6.color.a = 1.0;
  // Create the vertices for the points and lines

  marker_pub.publish(line_list6);
  /*
    // std_msgs::String result;
    // result.data = msg->x;
    unsigned char buff[1] = {0};
    buff[0] = steering_wheel_angle.data;
    // buff[1] = msg->y;
    // ROS_INFO("READ = :%s",buff[0]);
    ser.write(buff, 1);
  */
  //}

  // }
  loop_rate.sleep();
  ros::spinOnce();
}