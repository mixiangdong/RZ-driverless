#	include	<ros/ros.h>
#	include	"rfans_driver/null.h"
int	main(int argc, char **argv)
{
    ros::init(argc,	argv, "client");//	初始化,节点命名为"client"
	ros::NodeHandle	nhh;
	ros::ServiceClient client =	nhh.serviceClient<rfans_driver::null>("greetings");
	//	定义service客户端,service名字为“greetings”,service类型为Service_demo
	//	实例化srv,设置其request消息的内容,这里request包含两个变量,name和age,见Greeting.srv
	rfans_driver::null srv;
	srv.request.state= 1;
	client.call(srv);
    return	0;
}