//修改数据类型，GPS坐标转化的DOUBLE	和 double64
//弄清楚GPS数据形式修改方向差的BUG
//修改头文件
#include "rfans_driver/steering_wheel_angle.h"
#include <math.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <stdio.h>
#include <unistd.h>
#include "rfans_driver/Point.h"
#include "rfans_driver/gps_data.h"
#include "rfans_driver/null.h"

#define  spoint struct point
#define lidar_center_dis 1
#define PI 3.1415926

struct point{
	double x;
	double y;
	double yaw;
};

typedef struct 
{
	double setValue; 
	double actualValue;
	double controlValue;
	double lastLastError;      
	double lastError;      
	double currentError;  
	double SigmaError; 
	double Kp,Ki,Kd;    
} pid_t_;


class Control
{
   private:
   spoint left1,left2,right1,right2,middle1,middle2,center;
   double k0,k,b,lateralerror,theta,a_fa,lateral,steering_wheel_angle;
   int point_state;
pid_t_ yaw_pid;//航向角偏差
pid_t_ lateralError_pid;//横向偏差
double pid_yaw;
double pid_lateraError;

   public:

   		//对航向角误差的PID参数初始化
		void PID_init_yaw(pid_t_ *pid)
		{
			pid->setValue =0.0;
	        pid->actualValue =0.0;
	        pid->controlValue =0.0;
	        pid->lastLastError =0.0;
	        pid->lastError =0.0;
			pid->currentError =0.0;
			pid->SigmaError =0.0;
			pid->Kp = 1*4;
			pid->Ki = 0;
			pid->Kd = 0.0;
		}
        
		//对横向误差的PID参数初始化
		void PID_init_lateralError(pid_t_ *pid)
		{
			pid->setValue =0.0;
	        pid->actualValue =0.0;
	        pid->controlValue =0.0;
	        pid->lastLastError =0.0;
	        pid->lastError =0.0;
			pid->currentError =0.0;
			pid->SigmaError =0.0;
			pid->Kp = 45*4;
			pid->Ki = 0;
			pid->Kd = 0.0;
		}

		//位置PID
        double PID2_realize(pid_t_ *pid,double setValue,double currentValue)
		{
			pid->actualValue = currentValue;
			pid->setValue = setValue;
			pid->currentError = pid->setValue - pid->actualValue;//当前误差，即设置值与当前值的差
			pid->SigmaError += pid->currentError;//σ误差，为积分项
	        if(fabs(pid->currentError)<0.10 )//如果当前误差小于0.1
		    pid->SigmaError =0.0;
	        pid->controlValue = 	pid->Kp*pid->currentError
						          + pid->Ki*pid->SigmaError
						          + pid->Kd*(pid->currentError - pid->lastError);
											
	        ROS_INFO("\ncurrentError=%f  lastError=%f  control_value=%f p=%.3f  i=%.3f  d=%.3f\n",
						pid->currentError,pid->lastError,pid->controlValue,pid->Kp,pid->Ki,pid->Kd);
						
	        pid->lastError = pid->currentError;
	        pid->lastLastError = pid->lastError;
	
	        return pid->controlValue;
		}

		//增量PID
        double PID1_realize(pid_t_ *pid,double setValue,double currentValue)
        {
			pid->actualValue = currentValue;
			pid->setValue = setValue;
			pid->currentError = pid->setValue - pid->actualValue;
			pid->controlValue +=  pid->Kp *(pid->currentError -pid->lastError)
						        + pid->Ki * pid->currentError
						        + pid->Kd * (pid->currentError - 2*pid->lastError +pid->lastLastError);
	        pid->lastError = pid->currentError;
			pid->lastLastError = pid->lastError;
			
			return pid->controlValue;
		}


int cross_multiply(spoint middle1,spoint middle2,spoint center)
{
	double x;
	x=(middle2.x-middle1.x)*(center.y-middle1.y)-(middle2.y-middle1.y)*(center.x-middle1.x);
	if(x>0)//点在左侧
	return 1;
	else if(x=0)//点在线上
	return 0;
	else
	return -1;//点在右侧
}

//求两点距离，前点指向后点。
void distance(spoint p1,spoint p2,double *dis)
{
	*dis=sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));
}

//求向量内积。函数体默认GPS正东为0度，形式可以为0-360，也可以为-180-+180。
void vector_multiply(double angle,spoint center,spoint location,double *multiply) {
	double arc, x1, x2, y1, y2,cha_x,cha_y,length;
	arc = angle / 180 * PI;
	x1 = cos(arc);
	y1 = sin(arc);
	cha_x = center.x - location.x;
	cha_y = center.y - location.y;
	length = sqrt(cha_x*cha_x + cha_y * cha_y);
	x2 = cha_x / length;
	y2 = cha_y / length;
	*multiply = x1 * x2 + y1 * y2;
}


        //求横向偏差，根据期望航向角（上一个锥桶或发车位置指向目标点的方向）、当前点指向目标点的方向以及当前点和目标点的距离计算
		//为-时，车在计划路径的右侧，（有待讨论符号问题）
		double LateralError(double t_yaw_start,double t_yaw_now,double dis2end)
		{
			return ((sin((t_yaw_start - t_yaw_now)*PI/180)) * dis2end);
		}

	void callback2(const rfans_driver::Point::ConstPtr &point_msg)
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
			
			ROS_INFO("left2.x: %f,left2.y: %f,left2.y: %f,right2.y: %f\n",left2.x,left2.y,left2.y,right2.y);
			ROS_INFO("left1.x: %f,left1.y: %f,left1.y: %f,right1.y: %f\n",left1.x,left1.y,right1.x,right1.y);
			ROS_INFO("middle1.x: %f,middle1.y: %f,middle2.x: %f,middle2.y: %f,\n\n\n",middle1.x,middle1.y,middle2.x,middle2.y);
		}

		
bool callback3(rfans_driver::null::Request &req,rfans_driver::null::Response &res)
		{
			
  if (req.state)
			{
				PID_init_yaw(&yaw_pid);
				PID_init_lateralError(&lateralError_pid);	

				for( ; ; )
				{
					sleep(1);//每次循环延迟1秒

					center.x=lidar_center_dis;
                    center.y=0;	
if (middle2.x == 1000 && middle2.y == 1000 && middle1.x != 1000 && middle1.y != 1000)
			{
				if (left1.y - right1.y == 0)
				{
					steering_wheel_angle = 1000;
				}

				else if (fabs(left1.x - right1.x)<0.001)
				{
					steering_wheel_angle = 0;
				}
				else
				{
					k0 = (left1.y - right1.y) / (left1.x - right1.x);
					k = -1 / k0;
					b = middle1.y - k*middle1.x;
					lateralerror = fabs(k*center.x + b - center.y) / sqrt(k*k + 1);
					theta = atan(k) / PI * 180;//如果为正，应该逆时针打方向，PID输出+，反之亦反

					lateral = -1 * lateralerror*cross_multiply(middle1, middle2, center);//如果为正，应该逆时针打方向，反之亦反

					steering_wheel_angle = PID2_realize(&yaw_pid, 0.0, theta) + PID2_realize(&lateralError_pid, 0.0, lateral);
				}

				point_state = 1;
			}
			else if (middle2.x != 1000 && middle2.y != 1000 && middle1.x != 1000 && middle1.y != 1000)
			{
				k = (middle1.y - middle2.y) / (middle1.x - middle2.x);
				b = (middle2.x*middle1.y - middle1.x*middle2.y) / (middle2.x - middle1.x);
				ROS_INFO("k: %.9f\n", k);
				ROS_INFO("b: %.9f\n", b);
				lateralerror = fabs(k*center.x + b - center.y) / sqrt(k*k + 1);
				theta = -1*atan(k) / PI * 180;//如果为-，应该逆时针打方向，反之亦反

				ROS_INFO("theta: %.9f\n", theta);

				lateral =  lateralerror*cross_multiply(middle1, middle2, center);//如果为-，应该逆时针打方向，反之亦反

				ROS_INFO("lateral: %.9f\n", lateral);

				pid_yaw = PID2_realize(&yaw_pid, 0.0, theta);
				pid_lateraError = PID2_realize(&lateralError_pid, 0.0, lateral);
				steering_wheel_angle = pid_yaw + pid_lateraError;

				point_state = 2;
			}
			else
			{
				steering_wheel_angle = steering_wheel_angle;
				pid_yaw = 1000;
				pid_lateraError = 1000;
				point_state = 0;
			}
			ROS_INFO("\npid_yaw: %.9f\n", pid_yaw);
			ROS_INFO("pid_lateraError: %.9f\n", pid_lateraError);
			ROS_INFO("steering_wheel_angle: %.9f\n\n\n", steering_wheel_angle);
			ROS_INFO("point_state: %d\n", point_state);
					ros::spinOnce();
                }
			}

			return true;
        } 

		void run()
		{
			ros::NodeHandle nh;
			ros::Subscriber sub2 = nh.subscribe("Point",1,&Control::callback2,this);
			ros::ServiceServer service = nh.advertiseService("greetings",&Control::callback3,this);
			//control_pub= nh.advertise<rfans_river::steering_wheel_angle>("steering_wheel_angle",1);
			
			ros::spin();
		}		
};



int main(int argc,char**argv)
{
	ros::init(argc,argv,"upper_control");
	
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