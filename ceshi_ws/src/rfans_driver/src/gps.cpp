      #include <ros/ros.h> 
      #include <serial/serial.h>  //ROS已经内置了的串口包 
      #include <std_msgs/String.h> 
      #include <std_msgs/Empty.h> 
      #include <string.h>
      #include <stdio.h>
      #include <stdlib.h>
      #include "rfans_driver/gps_data.h"
      using namespace std;
 
      serial::Serial ser; //声明串口对象 
      //回调函数 
     // void write_callback(const std_msgs::String::ConstPtr& msg) 
     // { 
     // ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
      //ser.write(msg->data);   //发送串口数据 
      //} 
      int main (int argc, char** argv) 
      { 
          //初始化节点 
           ros::init(argc, argv, "serial_example_node"); 
           //声明节点句柄 
           ros::NodeHandle nh; 
           //订阅主题，并配置回调函数 
           //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
           //发布主题 
           ros::Publisher read_pub = nh.advertise<rfans_driver::gps_data>("read", 1000); 
           try 
           { 
            //设置串口属性，并打开串口 
              ser.setPort("/dev/ttyUSB0"); 
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
            } 
            //指定循环的频率 
            rfans_driver::gps_data msg;
           ros::Rate loop_rate(20); 
           while(ros::ok()) 
           { 

                if(ser.available())
                { 
                     ROS_INFO_STREAM("Reading from serial port\n");  
                     std_msgs::String result; 
                     result.data = ser.read(ser.available()); 
                     ROS_INFO_STREAM("Read: " << result.data); 
                     char *s=(char*)result.data.c_str();
                     int in=0 ,j;
                     char *p[100];
                     char *outer_ptr = NULL;
                     *outer_ptr;
                     char *inner_ptr = NULL;
                     *inner_ptr;
                     //int count = 0;
                     float latitude_before,longitude_before,hgt_before,yaw_before;
                     // string INS_before,POS_before;
                     while((p[in]=strtok_r(s,"\n",&outer_ptr))!=NULL)
                      {
                          //count = 0;
                          s = p[in];
                          while((p[in]= strtok_r(s,",",&inner_ptr))!=NULL)
                          {
                             in++;
                             s = NULL;
                             //count++;
                             //count++;
                          }
                          s = NULL; 
                      }
                      s = NULL;
                      //if(count = 57)
                    
                   
                    if(strcmp(p[0],"#INSPVAXA")==0)  //有用
                          {
                              msg.latitude = atof(p[11]);
                              msg.longitude = atof(p[12]);
                         //     msg.hgt = atof(p[13]);
                              msg.yaw = atof(p[20]);
                       //       msg.INS = p[9];
                   //           msg.POS = p[10];
                            latitude_before =msg.latitude;
                            longitude_before=msg.longitude;
                     //       hgt_before=msg.hgt; 
                            yaw_before=msg.yaw;
                   //         INS_before=msg.INS;
                   //         POS_before=msg.POS;

                              ROS_INFO("AA:%f",msg.yaw);
                              //return 0;
                          }
                       //  }
                    else
                          {
                            msg.latitude =latitude_before;
                            msg.longitude=longitude_before;
                          //  msg.hgt=hgt_before;
                            msg.yaw=yaw_before;
                          //  msg.INS=INS_before;
                          //  msg.POS=POS_before;
                                 float cc = 522;
                                 ROS_INFO("AA:%f",cc);
                          }
                      //ROS_INFO("ccc:%s",p[12]);
                      /*if(count= 54)
                      {
                          if(p[0] = "#HEADINGA")
                          {
                              //msg.longitude = atof(p[37]);
                             // msg.latitude = atof(p[38]);
                              msg.yaw = atof(p[12]);
                              return 0;
                          }
                          else if(p[0]="#BESTPOSA")
                          {
                              //msg.longitude = atof(p[11]);
                              //msg.latitude = atof(p[12]);
                              msg.yaw = atof(p[1]);
                              return 0;
                          }
                      }
                      else
                      {
                        return 0;// break;                                                                                                                                                                                                                                       
                      }*/
                     //ROS_INFO("ZZZ:%f",msg.yaw);
                     // msg.longitude = atof(p[37]);
                     // msg.latitude = atof(p[38]);
                      //msg.yaw = atof(p[12]);
                 
                }   
               read_pub.publish(msg);    
            
  
              //处理ROS的信息，比如订阅消息,并调用回调函数 
            ros::spinOnce();  
            loop_rate.sleep(); 
} 
} 

