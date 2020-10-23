//修改数据类型，GPS坐标转化的DOUBLE	和 double64
//弄清楚GPS数据形式修改方向差的BUG
//修改头文件

#include "rfans_driver/Point.h"
#include "rfans_driver/gps_data.h"
#include "rfans_driver/null.h"
#include "rfans_driver/steering_wheel_angle.h"
#include <math.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
using namespace std;

#define spoint struct point
#define extreme_radius 18
#define small_radius_factor 0.5
#define large_radius_factor 0.5
#define radius_factor 1 //标准为1,越小，车子越靠近内侧。
#define PI 3.1415926
#define small_center_factor 1
#define large_center_factor 1
#define center_center_distance_DisThreshold 0.3
#define gps_lidar_L 0.8

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

class Control {
private:
  spoint points1, POINTS1, points2, POINTS2, POINTS_point1, POINTS_point2;
  spoint points11, POINTS11, points22, POINTS22, POINTS_point11, POINTS_point22;
  spoint left1, left1_, left2, left2_, right1, right1_, right2, right2_,
      middle1, middle1_, middle2, middle2_, now_location, now_location_;
  spoint a_left, a_right, a_middle, b_left, b_right, b_middle, c_left, c_right,
      c_middle, left_center, right_center, middle_center;
  spoint small_center, large_center;
  double dis_a, dis_A, dis_b, dis_B, dis_Aa, dis_Bb;
  double left_radius, right_radius, small_radius, large_radius, target_radius,
      multiply, middle_radius, target_vehicle_distance;
  double control_factor1, control_factor2, disThreshold1,
      disThreshold2; //控制control_factor1,control_factor2为零。
  double center_vehicle_distance, center_center_distance, cross_mult, cha,
      vector_mult, angle_target, angle_assist, angle_lateral, yaw_diff,
      lateralError, now_location_c_middle_dis, angle_90_, angle_;
  double pid_yaw, pid_lateral;
  int arrive_target_flag, target_vehicle_distance_flag_save,
      target_vehicle_distance_flag, start_sign, symbol_1, symbol_2;
  pid_t_ yaw_pid;          //航向角偏差
  pid_t_ lateralError_pid; //横向偏差
  int i1, i2, j1, j2, state;
  rfans_driver::steering_wheel_angle steering_wheel_angle;
  vector<spoint> left_points;
  vector<spoint> right_points;

public:
  ros::NodeHandle nhhh;
  ros::Publisher control_pub =
      nhhh.advertise<rfans_driver::steering_wheel_angle>("steering_wheel_angle",
                                                         1000);
  //对航向角误差的PID参数初始化
  void PID_init_yaw(pid_t_ *pid) {
    pid->setValue = 0.0;
    pid->actualValue = 0.0;
    pid->controlValue = 0.0;
    pid->lastLastError = 0.0;
    pid->lastError = 0.0;
    pid->currentError = 0.0;
    pid->SigmaError = 0.0;
    pid->Kp = 1 * 1.3 * 4;
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

  double K(spoint a, spoint b) {
    if (a.x == b.x) {
      return 10000;
    } else {
      return (b.y - a.y) / (b.x - a.x);
    }
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
    pid->controlValue = pid->Kp * pid->currentError +
                        pid->Ki * pid->SigmaError +
                        pid->Kd * (pid->currentError - pid->lastError);

    /* ROS_INFO("\ncurrentError=%f  lastError=%f  control_value=%f p=%.3f
       i=%.3f  "
            "d=%.3f\n",
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
  /*
    void swap(double &x, double &y) {
      double tmp = x;
      x = y;
      y = tmp;
    }
  */
  void swap(spoint x, spoint *y) {
    y->x = x.y;
    y->y = x.x;
    y->yaw = x.yaw;
  }
  //已知三个
  //点求圆心和半径。
  void get_center_radius(spoint px1, spoint px2, spoint px3, double *center_x,
                         double *center_y, double *R) {
    double x1, y1, x2, y2, x3, y3;
    double a, b, c, g, e, f;
    x1 = px1.x;
    y1 = px1.y;
    x2 = px2.x;
    y2 = px2.y;
    x3 = px3.x;
    y3 = px3.y;
    e = 2 * (x2 - x1);
    f = 2 * (y2 - y1);
    g = x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1;
    a = 2 * (x3 - x2);
    b = 2 * (y3 - y2);
    c = x3 * x3 - x2 * x2 + y3 * y3 - y2 * y2;
    *center_x = (g * b - c * f) / (e * b - a * f);
    *center_y = (a * g - c * e) / (a * f - b * e);
    *R = sqrt((*center_x - x1) * (*center_x - x1) +
              (*center_y - y1) * (*center_y - y1));
  }

  //向量运算符号与航向控制的符号关系函数；
  //规定从航向往圆心和车的向量偏转，+为逆时针，-为顺时针；
  //规定期望半径减去实际车子距离圆心的距离差值大于0时为+，小于0时为-；
  //规定为了使车子达到预期轨迹而控制其逆时针偏转为+，顺时针偏转为-;
  //其中第一个参数为下面规定的差乘，第二个参数可以为内积，也可以直接带入期望半径减去实际车子距离圆心的距离差。
  int symbol(double cross_m, double vector_m) {
    if (cross_m * vector_m >= 0)
      return 1;
    else
      return -1;
  }

  //向量叉乘,从航向往圆心和车的向量偏转，+为逆时针，-为顺时针。
  double cross_multiply(double angle, spoint center, spoint location) {
    double arc, x1, y1, cha_x, cha_y;
    arc = angle / 180 * PI;
    x1 = cos(arc);
    y1 = sin(arc);
    cha_x = center.x - location.x;
    cha_y = center.y - location.y;
    return (x1 * cha_y - y1 * cha_x);
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

  //已知两点求方向角,得到0~360的方向角，x轴为0.向y轴方向为正,返回单位为度
  double angle(double x1, double y1, double x2, double y2) {
    double xx, yy, angle_temp;

    xx = x2 - x1;
    yy = y2 - y1;

    if (xx == 0.0)
      angle_temp = PI / 2.0;
    else
      angle_temp = atan(fabs(yy / xx));

    if ((xx < 0.0) && (yy >= 0.0))
      angle_temp = PI - angle_temp;
    else if ((xx < 0.0) && (yy < 0.0))
      angle_temp = PI + angle_temp;
    else if ((xx >= 0.0) && (yy < 0.0))
      angle_temp = PI * 2.0 - angle_temp;
    else {
    }
    angle_temp = angle_temp * 180 / PI;

    return (angle_temp);
  }

  double cha_angle(double angle1, double angle2) {
    if (fabs(angle1 - angle2) < 180) {
      return angle1 - angle2;
    }

    else if ((angle1 - angle2) < 0) {
      return angle1 - angle2 + 360;
    } else
      return angle1 - angle2 - 360;
  }
  //求横向偏差，根据期望航向角（上一个锥桶或发车位置指向目标点的方向）、当前点指向目标点的方向以及当前点和目标点的距离计算
  //为-时，车在计划路径的右侧，（有待讨论符号问题）
  double LateralError(double t_yaw_start, double t_yaw_now, double dis2end) {
    return ((sin((t_yaw_start - t_yaw_now) * PI / 180)) * dis2end);
  }

  // double jd;   输入参数: 地理坐标的经度，以度为单位
  // double wd;   输入参数: 地理坐标的纬度，以度为单位
  // short  DH;   输入参数: 三度带或六度带的带号
  // short DH_width;   带宽，3或6
  // LP；
  // 判断中央经度是直接给出还是需要间接计算，-1000表示间接计算，其他数值表示直接的中央经度
  void GeoToGauss(double jd, double wd, short DH, short DH_width, double *y,
                  double *x, double LP) {
    double t;  //  t=tgB
    double L;  //  中央经线的经度
    double l0; //  经差，以度为单位，经度和中央经度的差
    double jd_hd, wd_hd; //  将jd、wd转换成以弧度为单位
    double et2;          //  et2 = (e' ** 2) * (cosB ** 2)
    double N;            //  N = C / sqrt(1 + et2)
    double
        X;                         //  克拉索夫斯基椭球中子午弧长，即地球上一点所在经线与赤道的交点距离该点在经线上的弧长
    double m;                      //  m = cosB * PI/180 * l0
    double tsin, tcos;             //  sinB,cosB
    double b_e2 = 0.0067385254147; //第二曲率半径的平方
    double b_c = 6399698.90178271; //极点处子午线曲率半径

    jd_hd = jd * PI / 180.0; // 将以秒为单位的经度转换成度为单位再转换为弧度
    wd_hd = wd * PI / 180.0; // 将以秒为单位的纬度转换成度为单位再转换为弧度

    // 如果不设中央经线（缺省参数: -1000），则计算中央经线，
    // 否则，使用传入的中央经线，不再使用带号和带宽参数
    // L = (DH - 0.5) * DH_width ;      // 计算中央经线的经度
    if (LP == -1000) {
      L = DH * DH_width; // 计算中央经线的经度
    } else {
      L = LP;
    }

    l0 = jd - L;       // 计算经差
    tsin = sin(wd_hd); // 计算sinB
    tcos = cos(wd_hd); // 计算cosB
    // 计算克拉索夫斯基椭球中子午弧长X
    X = 111134.8611 * wd -
        (32005.7799 * tsin + 133.9238 * pow(tsin, 3) + 0.6976 * pow(tsin, 5) +
         0.0039 * pow(tsin, 7)) *
            tcos;

    et2 = b_e2 * pow(tcos, 2); //  et2 = (e' ** 2) * (cosB ** 2)
    N = b_c / sqrt(1 + et2);   //  N = C / sqrt(1 + et2)
    t = tan(wd_hd);            //  t=tgB
    m = PI / 180 * l0 * tcos;  //  m = cosB * PI/180 * l0

    *x = X +
         N * t * (0.5 * pow(m, 2) +
                  (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) /
                      24.0 +
                  (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);
    *y = N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 +
              (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 -
               58.0 * et2 * pow(t, 2)) *
                  pow(m, 5) / 120.0);
  }

  void coordinate_transformation(double lidar_x, double lidar_y, double gps_x,
                                 double gps_y, double gps_yaw, double *global_x,
                                 double *global_y) {
    double arc_gps_yaw, translation_x, translation_y;
    arc_gps_yaw = gps_yaw * PI / 180; //实际顺时针旋转角
    double arc_gps_yaw_ = (gps_yaw - 90) * PI / 180;
    // arc_auxiliary=PI/2-arc_gps_yaw;//实际逆时针旋转角
    translation_x = gps_x + gps_lidar_L * cos(arc_gps_yaw); // x轴偏转量
    translation_y = gps_y + gps_lidar_L * sin(arc_gps_yaw); // y轴偏转量
    *global_x = -1 * lidar_x * sin(arc_gps_yaw_) + lidar_y * cos(arc_gps_yaw_) +
                translation_x;
    *global_y = lidar_x * cos(arc_gps_yaw_) + lidar_y * sin(arc_gps_yaw_) +
                translation_y;
  }

  //将gps的msg转换为高斯坐标系，传给外部变量now_location。
  void callback1(const rfans_driver::gps_data::ConstPtr &gps_msg) {
    double m, n, x, y;
    m = gps_msg->latitude;
    n = gps_msg->longitude;
    // ROS_INFO("a:%.9f,b:%.9f", m, n);

    //珠海是北纬21° 48′～22°27′、东经113°03′～114°19′之间,3度带的38带号
    //合肥经度117.25，3度带是39带号
    GeoToGauss(n, m, 39, 3, &y, &x, 117);
    // now_location.x = gps_msg->x;
    // now_location.y = gps_msg->y;
    now_location.x = x;
    now_location.y = y;
    now_location.yaw = gps_msg->yaw;

    // ROS_INFO("now_location.x: %.9f,now_location.y :%.9f,now_location.yaw
    // :%.9f",
    //        now_location.x, now_location.y, now_location.yaw);
  }
  /*
    void callback2(const rfans_driver::lidarpoint::ConstPtr &lidarpoint_msg) {

      points11.x = lidarpoint_msg->a_x;
      points11.y = lidarpoint_msg->a_y;
      points22.x = lidarpoint_msg->b_x;
      points22.y = lidarpoint_msg->b_y;
      POINTS11.x = lidarpoint_msg->A_X;
      POINTS11.y = lidarpoint_msg->A_Y;
      POINTS22.x = lidarpoint_msg->B_X;
      POINTS22.y = lidarpoint_msg->B_Y;
      POINTS_point11.x = lidarpoint_msg->Aa_Xx;
      POINTS_point11.y = lidarpoint_msg->Aa_Yy;
      POINTS_point22.x = lidarpoint_msg->Bb_Xx;
      POINTS_point22.y = lidarpoint_msg->Bb_Yy;

      if (points11.x != 1000 && points11.y != 1000) {
        coordinate_transformation(points11.x, points11.y, now_location.x,
                                  now_location.y, now_location.yaw, &points1.x,
                                  &points1.y);
        distance(now_location, points1, &dis_a);
      } else {
        dis_a = 1000;
        points1.x = 1000;
        points1.y = 1000;
      }
      if (points22.x != 1000 && points22.y != 1000) {
        coordinate_transformation(points22.x, points22.y, now_location.x,
                                  now_location.y, now_location.yaw, &points2.x,
                                  &points2.y);
        distance(now_location, points2, &dis_b);
      } else {
        dis_b = 1000;
        points2.x = 1000;
        points2.y = 1000;
      }
      if (POINTS11.x != 1000 && POINTS11.y != 1000) {
        coordinate_transformation(POINTS11.x, POINTS11.y, now_location.x,
                                  now_location.y, now_location.yaw, &POINTS1.x,
                                  &POINTS1.y);
        distance(now_location, POINTS1, &dis_A);
      } else {
        dis_A = 1000;
        POINTS1.x = 1000;
        POINTS1.y = 1000;
      }
      if (POINTS22.x != 1000 && POINTS22.y != 1000) {
        coordinate_transformation(POINTS22.x, POINTS22.y, now_location.x,
                                  now_location.y, now_location.yaw, &POINTS2.x,
                                  &POINTS2.y);
        distance(now_location, POINTS2, &dis_B);
      } else {
        dis_B = 1000;
        POINTS2.x = 1000;
        POINTS2.y = 1000;
      }
      if (POINTS_point11.x != 1000 && POINTS_point11.y != 1000) {
        coordinate_transformation(
            POINTS_point11.x, POINTS_point11.y, now_location.x, now_location.y,
            now_location.yaw, &POINTS_point1.x, &POINTS_point1.y);
        distance(now_location, POINTS_point1, &dis_Aa);
      } else {
        dis_Aa = 1000;
        POINTS_point1.x = 1000;
        POINTS_point1.y = 1000;
      }
      if (POINTS_point22.x != 1000 && POINTS_point22.y != 1000) {
        coordinate_transformation(
            POINTS_point22.x, POINTS_point22.y, now_location.x, now_location.y,
            now_location.yaw, &POINTS_point2.x, &POINTS_point2.y);
        distance(now_location, POINTS_point2, &dis_Bb);
      } else {
        dis_Bb = 1000;
        POINTS_point2.x = 1000;
        POINTS_point2.y = 1000;
      }

      if (dis_a < dis_b) {
        right1.x = points1.x;
        right2.x = points2.x;
        right1.y = points1.y;
        right2.y = points2.y;

      } else {
        right1.x = points2.x;
        right2.x = points1.x;
        right1.y = points2.y;
        right2.y = points1.y;
      }
      if (dis_A < dis_B) {
        left1.x = POINTS1.x;
        left2.x = POINTS2.x;
        left1.y = POINTS1.y;
        left2.y = POINTS2.y;

      } else {
        left1.x = POINTS2.x;
        left2.x = POINTS1.x;
        left1.y = POINTS2.y;
        left2.y = POINTS1.y;
      }
          middle1.x=(left1.x+right1.x)/2;
          middle1.y=(left1.y+right1.y)/2;
          middle2.x=(left2.x+right2.x)/2;
          middle2.y=(left2.y+right2.y)/2;

      ROS_INFO("left2.x: %f,left2.y: %f,left2.y: %f,right2.y: %f\n", left2.x,
               left2.y, left2.y, right2.y);
      ROS_INFO("left1.x: %f,left1.y: %f,left1.y: %f,right1.y: %f\n", left1.x,
               left1.y, right1.x, right1.y);
      ROS_INFO("middle1.x: %f,middle1.y: %f,middle2.x: %f,middle2.y: %f\n",
               middle1.x, middle1.y, middle2.x, middle2.y);
    }
    */
  void callback2(const rfans_driver::Point::ConstPtr &lidarpoint_msg) {

    points11.x = lidarpoint_msg->a2;
    points11.y = lidarpoint_msg->b2;
    points22.x = lidarpoint_msg->a4;
    points22.y = lidarpoint_msg->b4;
    POINTS11.x = lidarpoint_msg->a1;
    POINTS11.y = lidarpoint_msg->b1;
    POINTS22.x = lidarpoint_msg->a3;
    POINTS22.y = lidarpoint_msg->b3;

    if (points11.x != 1000 && points11.y != 1000) {
      coordinate_transformation(points11.x, points11.y, now_location.x,
                                now_location.y, now_location.yaw, &points1.x,
                                &points1.y);
      distance(now_location, points1, &dis_a);
    } else {
      dis_a = 1000;
      points1.x = 1000;
      points1.y = 1000;
    }
    if (points22.x != 1000 && points22.y != 1000) {
      coordinate_transformation(points22.x, points22.y, now_location.x,
                                now_location.y, now_location.yaw, &points2.x,
                                &points2.y);
      distance(now_location, points2, &dis_b);
    } else {
      dis_b = 1000;
      points2.x = 1000;
      points2.y = 1000;
    }
    if (POINTS11.x != 1000 && POINTS11.y != 1000) {
      coordinate_transformation(POINTS11.x, POINTS11.y, now_location.x,
                                now_location.y, now_location.yaw, &POINTS1.x,
                                &POINTS1.y);
      distance(now_location, POINTS1, &dis_A);
    } else {
      dis_A = 1000;
      POINTS1.x = 1000;
      POINTS1.y = 1000;
    }
    if (POINTS22.x != 1000 && POINTS22.y != 1000) {
      coordinate_transformation(POINTS22.x, POINTS22.y, now_location.x,
                                now_location.y, now_location.yaw, &POINTS2.x,
                                &POINTS2.y);
      distance(now_location, POINTS2, &dis_B);
    } else {
      dis_B = 1000;
      POINTS2.x = 1000;
      POINTS2.y = 1000;
    }

    if (dis_a < dis_b) {
      right1.x = points1.x;
      right2.x = points2.x;
      right1.y = points1.y;
      right2.y = points2.y;

    } else {
      right1.x = points2.x;
      right2.x = points1.x;
      right1.y = points2.y;
      right2.y = points1.y;
    }
    if (dis_A < dis_B) {
      left1.x = POINTS1.x;
      left2.x = POINTS2.x;
      left1.y = POINTS1.y;
      left2.y = POINTS2.y;

    } else {
      left1.x = POINTS2.x;
      left2.x = POINTS1.x;
      left1.y = POINTS2.y;
      left2.y = POINTS1.y;
    }
    /*
        ROS_INFO("left2.x: %f,left2.y: %f,left2.y: %f,right2.y: %f\n", left2.x,
                 left2.y, left2.y, right2.y);
        ROS_INFO("left1.x: %f,left1.y: %f,left1.y: %f,right1.y: %f\n", left1.x,
                 left1.y, right1.x, right1.y);
                 */
    // ROS_INFO("middle1.x: %f,middle1.y: %f,middle2.x: %f,middle2.y: %f\n",
    //          middle1.x, middle1.y, middle2.x, middle2.y);
  }
  /*	void callback2(const rfans_driver::lidar::ConstPtr &point_msg)
                  {
                          left1.x = point_msg->a1;
                          left1.y = point_msg->b1;
                          left2.x = point_msg->a3;
                          left2.y = point_msg->b3;
                          right1.x = point_msg->a2;
                          right1.y = point_msg->b2;
                          right2.x = point_msg->a4;
                          right2.y = point_msg->b4;
                          middle1.x= point_msg->x1;
                          middle1.y= point_msg->y1;
                          middle2.x= point_msg->x2;
                          middle2.y= point_msg->y2;

                          ROS_INFO("left2.x: %f,left2.y: %f,left2.y:
     %f,right2.y: %f\n",left2.x,left2.y,left2.y,right2.y);
                          ROS_INFO("left1.x: %f,left1.y: %f,left1.y:
     %f,right1.y: %f\n",left1.x,left1.y,right1.x,right1.y);
                          ROS_INFO("middle1.x: %f,middle1.y: %f,middle2.x:
     %f,middle2.y: %f,\n\n\n",middle1.x,middle1.y,middle2.x,middle2.y);
                  }
  */

  bool callback3(rfans_driver::null::Request &req,
                 rfans_driver::null::Response &res) {

    if (req.state) {
      PID_init_yaw(&yaw_pid);
      PID_init_lateralError(&lateralError_pid);

      for (;;) {
        sleep(1); //每次循环延迟1秒
        i1 = 0;
        i2 = 0;
        j1 = 0;
        j2 = 0;
        /*
        swap(left1.x, left1.y);
        swap(left2.x, left2.y);
        swap(right1.x, right1.y);
        swap(right2.x, right2.y);
        swap(middle1.x, middle1.y);
        swap(middle2.x, middle2.y);
        swap(now_location.x, now_location.y);
*/
        swap(left1, &left1_);
        swap(left2, &left2_);
        swap(right1, &right1_);
        swap(right2, &right2_);
        swap(middle1, &middle1_);
        swap(middle2, &middle2_);
        swap(now_location, &now_location_);
        for (i1 = 0; i1 < left_points.size(); i1++) {
          if (fabs(left1_.x - left_points[i1].x) > 0.5 &&
              fabs(left1_.y - left_points[i1].y) > 0.5 && left1_.x != 1000 &&
              left1_.y != 1000) {
            continue;
          } else {
            break;
          }
        }
        if (i1 >= left_points.size()) {
          left_points.push_back(left1_);
        }

        for (i2 = 0; i2 < left_points.size(); i2++) {
          if (fabs(left2_.x - left_points[i2].x) > 0.5 &&
              fabs(left2_.y - left_points[i2].y) > 0.5 && left2_.x != 1000 &&
              left2_.y != 1000) {
            continue;
          } else {
            break;
          }
        }
        if (i2 >= left_points.size()) {
          left_points.push_back(left2_);
        }

        for (j1 = 0; j1 < right_points.size(); j1++) {
          if (fabs(right1_.x - right_points[j1].x) > 0.5 &&
              fabs(right1_.y - right_points[j1].y) > 0.5 && right1_.x != 1000 &&
              right1_.y != 1000) {
            continue;
          } else {
            break;
          }
        }
        if (j1 >= right_points.size()) {
          right_points.push_back(right1_);
        }

        for (j2 = 0; j2 < right_points.size(); j2++) {
          if (fabs(right2_.x - right_points[j2].x) > 0.5 &&
              fabs(right2_.y - right_points[j2].y) > 0.5 && right2_.x != 1000 &&
              right2_.y != 1000) {
            continue;
          } else {
            break;
          }
        }
        if (j2 >= right_points.size()) {
          right_points.push_back(right2_);
        }

        cout << "left_points_num: " << left_points.size() << endl;
        cout << "right_points_num: " << right_points.size() << endl;

        int left_points_num = left_points.size();
        int right_points_num = right_points.size();
        if (left_points_num > 3 && right_points_num > 3) {
          /*
          c_left = left_points[left_points_num - 1];
          b_left = left_points[left_points_num - 2];
          a_left = left_points[left_points_num - 3];

          c_right = right_points[right_points_num - 1];
          b_right = right_points[right_points_num - 2];
          a_right = right_points[right_points_num - 3];
*/
          c_left = left_points[left_points_num - 2];
          b_left = left_points[left_points_num - 3];
          a_left = left_points[left_points_num - 4];

          c_right = right_points[right_points_num - 2];
          b_right = right_points[right_points_num - 3];
          a_right = right_points[right_points_num - 4];

          if (fabs(atan(K(a_left, b_left)) - atan(K(a_left, c_left))) < 0.01) {
            left_radius = 1000;
          } else {
            get_center_radius(a_left, b_left, c_left, &left_center.x,
                              &left_center.y, &left_radius);
          }
          if (fabs(atan(K(a_right, b_right)) - atan(K(a_right, c_right))) <
              0.01) {
            right_radius = 1000;
          } else {
            get_center_radius(a_right, b_right, c_right, &right_center.x,
                              &right_center.y, &right_radius);
          }

          // ROS_INFO("atan(K(a_left, b_left)): %.9f\n", atan(K(a_left,
          // b_left)));
          // ROS_INFO("atan(K(a_left, c_left)): %.9f\n", atan(K(a_left,
          // c_left)));
          // ROS_INFO(" %.9f\n", (fabs(atan(K(a_left, b_left)) - atan(K(a_left,
          // c_left)))));
          // ROS_INFO("left_radius: %.9f\n", left_radius);
          // ROS_INFO("right_radius: %.9f\n", right_radius);

          if (left_radius <= extreme_radius || right_radius <= extreme_radius) {
            //不进行配对的圆弧轨迹。
            if (left_radius < right_radius) {
              small_radius = left_radius;
              small_center.x = left_center.x;
              small_center.y = left_center.y;
              large_radius = right_radius;
              large_center.x = right_center.x;
              large_center.y = right_center.y;
            } else {
              small_radius = right_radius;
              small_center.x = right_center.x;
              small_center.y = right_center.y;
              large_radius = left_radius;
              large_center.x = left_center.x;
              large_center.x = left_center.y;
            }

            distance(small_center, large_center, &center_center_distance);
            if (center_center_distance <= center_center_distance_DisThreshold) {
              target_radius = small_radius_factor * small_radius +
                              large_radius_factor * large_radius;

              middle_center.x = (small_center_factor * small_center.x +
                                 large_center_factor * large_center.x) /
                                2;
              middle_center.y = (small_center_factor * small_center.y +
                                 large_center_factor * large_center.y) /
                                2;
            } else {
              target_radius = small_radius + 1.35;

              middle_center.x = small_center.x;
              middle_center.y = small_center.y;
            }

            //配对后的圆弧轨迹。

            // get_center_radius(a_middle, b_middle, c_middle, &middle_center.x,
            //	&middle_center.y, &middle_radius);

            // ROS_INFO("middle_radius: %.9f\n", middle_radius);

            // target_radius = radius_factor * middle_radius;

            distance(now_location_, middle_center, &center_vehicle_distance);
            // ROS_INFO("center_vehicle_distance: %.9f\n",
            // center_vehicle_distance);
            cha = target_radius - center_vehicle_distance;
            cross_mult = cross_multiply((90 - now_location_.yaw), middle_center,
                                        now_location_);
            // ROS_INFO("cross_mult: %.9f\n", cross_mult);
            vector_multiply((90 - now_location_.yaw), middle_center,
                            now_location_, &multiply);
            ROS_INFO("multiply: %.9f\n", multiply);
            vector_mult = multiply;
            angle_90_ = acos(vector_mult) * 180 / PI;
            ROS_INFO("vector_mult: %.9f\n", vector_mult);
            ROS_INFO("angle_90_: %.9f\n", angle_90_);
            angle_ = 90 - angle_90_;

            ROS_INFO("angle_: %.9f\n", angle_);
            symbol_1 = symbol(cross_mult, cha);
            control_factor2 =
                symbol_1 *
                fabs(cha); //若为-，则应该控制车子逆时针转向，即左转;反之亦反.

            symbol_2 = symbol(cross_mult, vector_mult);
            control_factor1 =
                symbol_2 *
                fabs(
                    angle_); //若为+，则应该控制车子逆时针转向，即左转;反之亦反.
            state = 1;

          } else {

            a_middle.x = (a_left.x + a_right.x) / 2;
            b_middle.x = (b_left.x + b_right.x) / 2;
            c_middle.x = (c_left.x + c_right.x) / 2;
            a_middle.y = (a_left.y + a_right.y) / 2;
            b_middle.y = (b_left.y + b_right.y) / 2;
            c_middle.y = (c_left.y + c_right.y) / 2;
            //伪直线运动控制算法。
            angle_target =
                angle(b_middle.x, b_middle.y, c_middle.x, c_middle.y);
            // yaw_diff = angle_target - (90-now_location_.yaw);
            // //-时需要顺时针转向
            yaw_diff = cha_angle(angle_target, 90 - now_location_.yaw);
            control_factor1 = (-1 * yaw_diff); //改变方向，统一符号
            // ROS_INFO("yaw_diff: %.9f\n", yaw_diff);
            angle_assist =
                angle(a_middle.x, a_middle.y, c_middle.x, c_middle.y);
            if (fabs(angle_assist - angle_target) < 180) //防止角度边界处出错
            {
              angle_lateral = 0.5 * (angle_assist + angle_target);
            } else {
              angle_lateral = angle_target;
            }

            double angle_c =
                angle(now_location_.x, now_location_.y, c_middle.x, c_middle.y);
            ROS_INFO("angle_c: %.9f\n", angle_c);
            distance(now_location_, c_middle, &now_location_c_middle_dis);
            lateralError =
                LateralError(angle_lateral, angle_c,
                             now_location_c_middle_dis); //+时需要顺时针旋转
            control_factor2 = 1 * lateralError; //改变方向，统一符号
            state = 0;
          }
          ROS_INFO(
              "c_left.x: %.9f c_left.y: %.9f c_right.x: %.9f c_right.y: %.9f\n",
              c_left.x, c_left.y, c_right.x, c_right.y);
          ROS_INFO(
              "b_left.x: %.9f b_left.y: %.9f b_right.x: %.9f b_right.y: %.9f\n",
              b_left.x, b_left.y, b_right.x, b_right.y);
          ROS_INFO("a_left.x: %.9f a_left.y: %.9f a_right.x: %.9f a_right.y: "
                   "%.9f\n\n\n",
                   a_left.x, a_left.y, a_right.x, a_right.y);
        } else {
          c_left = left_points[left_points_num - 1];
          b_left = left_points[left_points_num - 2];

          c_right = right_points[right_points_num - 1];
          b_right = right_points[right_points_num - 2];

          c_middle.x = 0.5 * (c_left.x + c_right.x);
          c_middle.y = 0.5 * (c_left.y + c_right.y);
          b_middle.x = 0.5 * (b_left.x + b_right.x);
          b_middle.y = 0.5 * (b_left.y + b_right.y);

          angle_target = angle(b_middle.x, b_middle.y, c_middle.x, c_middle.y);
          // yaw_diff = angle_target - (90-now_location_.yaw);
          // //-时需要顺时针转向
          yaw_diff = cha_angle(angle_target, 90 - now_location_.yaw);
          control_factor1 = (-1 * yaw_diff); //改变方向，统一符号

          // angle_assist = angle(a_middle.x, a_middle.y, c_middle.x,
          // c_middle.y);
          // angle_lateral = 0.5 * (angle_assist + angle_target);

          double angle_c =
              angle(now_location_.x, now_location_.y, c_middle.x, c_middle.y);

          distance(now_location_, c_middle, &now_location_c_middle_dis);
          lateralError =
              LateralError(angle_target, angle_c,
                           now_location_c_middle_dis); //+时需要顺时针旋转
          control_factor2 = 1 * lateralError; //改变方向，统一符号
          state = 0;
          ROS_INFO(
              "c_left.x: %.9f c_left.y: %.9f c_right.x: %.9f c_right.y: %.9f\n",
              c_left.x, c_left.y, c_right.x, c_right.y);
          ROS_INFO(
              "b_left.x: %.9f b_left.y: %.9f b_right.x: %.9f b_right.y: %.9f\n",
              b_left.x, b_left.y, b_right.x, b_right.y);
        }

        ROS_INFO("left2.x: %f,left2.y: %f,right2.x: %f,right2.y: %f\n",
                 left2_.x, left2_.y, right2_.x, right2_.y);
        ROS_INFO("left1.x: %f,left1.y: %f,right1.x: %f,right1.y: %f\n",
                 left1_.x, left1_.y, right1_.x, right1_.y);
        ROS_INFO(
            "now_location.x: %.9f,now_location.y :%.9f,now_location.yaw :%.9f",
            now_location_.x, now_location_.y, now_location_.yaw);
        ROS_INFO("state: %d\n\n\n", state);
        ROS_INFO("control_factor1: %.9f\n", control_factor1);
        ROS_INFO("control_factor2: %.9f\n\n\n", control_factor2);

        // PID输出符号与control_factor符号相反
        pid_yaw = PID2_realize(&yaw_pid, 0.0, control_factor1);
        pid_lateral = PID2_realize(&lateralError_pid, 0.0, control_factor2);
        steering_wheel_angle.data = pid_yaw + pid_lateral;

        ROS_INFO("pid_yaw: %.9f", pid_yaw);
        ROS_INFO("pid_lateral: %.9f", pid_lateral);
        ROS_INFO("steering_wheel_angle.data: %.9f\n--------------------",
                 steering_wheel_angle.data);
        control_pub.publish(steering_wheel_angle);
        ros::spinOnce();
      }
      return true;
    }
  }
  void run() {
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe("read", 5, &Control::callback1, this);
    ros::Subscriber sub2 = nh.subscribe("Point", 5, &Control::callback2, this);
    ros::ServiceServer service =
        nh.advertiseService("greetings", &Control::callback3, this);
    // ros::Publisher control_pub=
    // nh.advertise<rfans_driver::steering_wheel_angle>("steering_wheel_angle",1);

    ros::spin();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "upper_control");

  /*try
  {
          //设置串口属性，并打开串口
          ser.setPort("/dev/ttyUSB1");
          ser.setBaudrate(115200);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
  }
  catch (serial::IOException& e)
  {
          ROS_ERROR_STREAM("Unable to open port ");
          return -1;
  }
  //检测串口是否已经打开，并给出提示信息
  if(ser.isOpen())
  {
          ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
          return -1;

  } */

  Control zh;

  zh.run();

  return 0;
}