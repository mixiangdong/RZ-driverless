#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <sensor_msgs/PointCloud2.h>
#include <rfans_driver/Point.h>
#include <rfans_driver/gps_data.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
//#include <time.h>

//定义锥桶及中心点坐标
const double PI = 3.141592653589;

double a1, a2, a3, a4, b1, b2, b3, b4;

class SubscribeAndPublish1
{
public:
  SubscribeAndPublish1()
  {
    //Topic you want to publish
    pub_point = n.advertise<rfans_driver::Point>("Point", 1);//锥桶中心点坐标 发给决策

    pub_cloud_output_centroid = n.advertise<sensor_msgs::PointCloud2> ("centroid_point", 1);//配对后锥桶及中心点 可视化

    //Topic you want to subscribe
    //sub_gps = n.subscribe("read", 1, &SubscribeAndPublish1::gps_callback, this);
    //sub_final_PointCloud2 = n.subscribe("cloud_cluster_big", 1, &SubscribeAndPublish1::cluster_big_callback, this);//未处理的大锥桶点云
    sub_final_PointCloud2 = n.subscribe("cloud_other", 1, &SubscribeAndPublish1::cluster_small_callback, this);//处理后的小锥桶点云
  }

  void cluster_small_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    clock_t start, end;
    start = clock();

    // 将msg点云格式为sensor_msgs/PointCloud2 格式转为 pcl/PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud);

    //欧式聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);;
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;				       	//创建点云索引向量，用于存储实际的点云信息
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;	                //欧式聚类对象
    ec.setClusterTolerance (0.3);								    	//设置近邻搜索的搜索半径，这里单位为m
    ec.setMinClusterSize (5);								            //设置一个聚类需要的最少的点数目为2000
    ec.setMaxClusterSize (1000);								      	//设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree);								        	//设置点云的搜索机制
    ec.setInputCloud (cloud);					                        //输入需要聚类的点云指针地址
    ec.extract (cluster_indices);								      	//得到2000-25000点的聚类

    std::cerr << "聚类数量: " << cluster_indices.size() << "\n" << std::endl;

    //为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中
    //迭代访问点云索引cluster_indices,直到分割出所有聚类
    int cluster_count = 0;
    int j = 0;
    double angle;       //angle为角度制
    std::vector<double> centroid_x, centroid_y, centroid_dis, dis_backup, centroid_angle, use_angle;
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        //循环容器中的点云的索引，并且分开保存索引的点云
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        cloud_cluster->points.push_back(cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ min;//用于存放三个轴的最小值
        pcl::PointXYZ max;//用于存放三个轴的最大值
        pcl::getMinMax3D(*cloud_cluster,min,max);//得到点云x,y,z三个轴上的最大值和最小值
            cluster_count++;
            double sum_x = 0,sum_y = 0;
            for(int i = 0; i < cloud_cluster->points.size(); i++)
            {
                sum_x += cloud_cluster->points[i].x;
                sum_y += cloud_cluster->points[i].y;
            }

            std::cerr << "Cluster " << cluster_count << " has " << cloud_cluster->points.size () << " data points." << std::endl;
            centroid_x.push_back( sum_x / cloud_cluster->points.size() );
            centroid_y.push_back( sum_y / cloud_cluster->points.size() );
            std::cerr << "The centroid of cluster: x = " << centroid_x[j] << ", y = " << centroid_y[j] << std::endl;

            double distance;
            distance =  std::sqrt( std::pow(centroid_x[j], 2) + std::pow(centroid_y[j], 2) );
            centroid_dis.push_back(distance); 
            dis_backup.push_back(distance);
            //std::cerr << "The distance of cluster " << " : " << centroid_dis[j] << std::endl;

            angle = std::atan( centroid_y[j] / centroid_x[j] ); //这里暂时是弧度
            angle = angle * 180 / PI + 180;
            centroid_angle.push_back(angle);
            //std::cerr << "The angle of centroid:  " << centroid_angle[j] << "°" << "\n" << std::endl;
            j++;
    }
    //std::cerr << "The total numeber of cluster before:" << cluster_count << "\n" << std::endl;

    //打印点坐标
    /*for(int i = 0; i <  cloud_output->points.size (); i++)
    {
         std::cerr << "point_" << i << ": x =  " << cloud_output->points[i].x << ", y = " << cloud_output->points[i].y
          << " , z = " << cloud_output->points[i].z << "\n" << std::endl;
    }
*/

    double x_1, y_1;
    switch(cluster_count)
    { 
      case 0: case 1:
      {
        //std::cerr << "The total numeber of cluster :" << cluster_count << " , loop break \n" << std::endl;
        
        rfans_driver::Point Point_msg;

        Point_msg.a1 = 1000;
        Point_msg.a2 = 1000;
        Point_msg.a3 = 1000;
        Point_msg.a4 = 1000;
        Point_msg.b1 = 1000;
        Point_msg.b2 = 1000;
        Point_msg.b3 = 1000;
        Point_msg.b4 = 1000;

        Point_msg.x1 = 1000;
        Point_msg.y1 = 1000;
        Point_msg.x2 = 1000;
        Point_msg.y2 = 1000;
        pub_point.publish(Point_msg);
        std::cerr << std::fixed << "Point_msg : (x1 = " << Point_msg.x1 << ", y1 = " << Point_msg.y1 <<
         ", Point_msg.x2 = " << Point_msg.x2 << ", Point_msg.y2 = " << Point_msg.y2 << "\n" << std::endl;

        x_1 = Point_msg.x1;
        y_1 = Point_msg.y1;
        
        end = clock();														//计时结束
        //std::cerr << "run time:" << (double)(end-start) / CLOCKS_PER_SEC << "s" << std::endl;
        break;
      }

      case 2:
      {
        //std::cerr << "The total numeber of cluster :" << cluster_count << " , loop break \n" << std::endl;
        
        double dis;
        dis = std::sqrt( std::pow(centroid_x[0] - centroid_x[1], 2) + std::pow(centroid_y[0] - centroid_y[1], 2) );
        if ( dis < 3.9 && abs(centroid_dis[0] - centroid_dis[1]) < 1.5 ) 
        {
            double x_ave, y_ave;
            x_ave = (centroid_x[0] + centroid_x[1]) / 2;
            y_ave = (centroid_y[0] + centroid_y[1]) / 2;

            double x1, y1;
            //conversion(gps_pionts, x_ave / 2, y_ave / 2, &x1, &y1, 1.5);
            rfans_driver::Point Point_msg;

            Point_msg.a1 = centroid_x[0];
            Point_msg.a2 = centroid_x[1];
            Point_msg.b1 = centroid_y[0];
            Point_msg.b2 = centroid_y[1];
            Point_msg.a3 = 1000;
            Point_msg.a4 = 1000;
            Point_msg.b3 = 1000;
            Point_msg.b4 = 1000;

            //Point_msg.x1 = x1;
            //Point_msg.y1 = y1;
            Point_msg.x1 = x_ave;
            Point_msg.y1 = y_ave;
            Point_msg.x2 = 1000;
            Point_msg.y2 = 1000;
            pub_point.publish(Point_msg);

            x_1 = Point_msg.x1;
            y_1 = Point_msg.y1;

            //std::cerr << "The centroid point: (x = " << x_ave << ", y = " << y_ave << ")" << std::endl;
            
            std::cerr << std::fixed << "Point_msg : (x1 = " << Point_msg.x1 << ", y1 = " << Point_msg.y1 <<
            ", Point_msg.x2 = " << Point_msg.x2 << ", Point_msg.y2 = " << Point_msg.y2 << "\n" << std::endl;
        }
        else
        {
            rfans_driver::Point Point_msg;

            Point_msg.a1 = 1000;
            Point_msg.a2 = 1000;
            Point_msg.a3 = 1000;
            Point_msg.a4 = 1000;
            Point_msg.b1 = 1000;
            Point_msg.b2 = 1000;
            Point_msg.b3 = 1000;
            Point_msg.b4 = 1000;

            Point_msg.x1 = 1000;
            Point_msg.y1 = 1000;
            Point_msg.x2 = 1000;
            Point_msg.y2 = 1000;
            pub_point.publish(Point_msg);

            x_1 = Point_msg.x1;
            y_1 = Point_msg.y1;

            
            std::cerr << std::fixed << "Point_msg : (x1 = " << Point_msg.x1 << ", y1 = " << Point_msg.y1 <<
            ", Point_msg.x2 = " << Point_msg.x2 << ", Point_msg.y2 = " << Point_msg.y2 << "\n" << std::endl;

        }
        
        end = clock();														//计时结束
        //std::cerr << "run time:" << (double)(end-start) / CLOCKS_PER_SEC << "s" << std::endl;        
        break; 
      }

      case 3:
      {
        //std::cerr << "The total numeber of cluster :" << cluster_count << " , loop break \n" << std::endl;

        //std::cerr << "排序前： " << centroid_dis[0] << " " << centroid_dis[1] << " " << centroid_dis[2] << std::endl;
        //std::cerr << "排序前： " << centroid_x[0] << " " << centroid_x[1] << " " << centroid_x[2] << std::endl;
        //std::cerr << "排序前： " << centroid_y[0] << " " << centroid_y[1] << " " << centroid_y[2] << "\n" << std::endl;

        double x, y, dis;//按距离从大到小对三个锥桶进行排序
        if(centroid_dis[0] < centroid_dis[1])
        {
            dis = centroid_dis[0];
            centroid_dis[0] = centroid_dis[1];
            centroid_dis[1] = dis;
            x = centroid_x[0];
            centroid_x[0] = centroid_x[1];
            centroid_x[1] = x;
            y = centroid_y[0];
            centroid_y[0] = centroid_y[1];
            centroid_y[1] = y;
        }
        if(centroid_dis[0] < centroid_dis[2])
        {
            dis = centroid_dis[0];
            centroid_dis[0] = centroid_dis[2];
            centroid_dis[2] = dis;
            x = centroid_x[0];
            centroid_x[0] = centroid_x[2];
            centroid_x[2] = x;
            y = centroid_y[0];
            centroid_y[0] = centroid_y[2];
            centroid_y[2] = y;
        }
        if(centroid_dis[1] < centroid_dis[2])
        {
            dis = centroid_dis[1];
            centroid_dis[1] = centroid_dis[2];
            centroid_dis[2] = dis;
            x = centroid_x[1];
            centroid_x[1] = centroid_x[2];
            centroid_x[2] = x;
            y = centroid_y[1];
            centroid_y[1] = centroid_y[2];
            centroid_y[2] = y;
        }

        //std::cerr << "排序后： " << centroid_dis[0] << " " << centroid_dis[1] << " " << centroid_dis[2] << std::endl;
        //std::cerr << "排序后： " << centroid_x[0] << " " << centroid_x[1] << " " << centroid_x[2] << std::endl;
        //std::cerr << "排序后： " << centroid_y[0] << " " << centroid_y[1] << " " << centroid_y[2] << "\n" << std::endl;

        double f_ave_x = 0, f_ave_y = 0;
        double dis01, dis02, dis12;
        dis01 = std::sqrt( std::pow(centroid_x[0] - centroid_x[1], 2) + std::pow(centroid_y[0] - centroid_y[1], 2) );
        dis02 = std::sqrt( std::pow(centroid_x[0] - centroid_x[2], 2) + std::pow(centroid_y[0] - centroid_y[2], 2) );
        dis12 = std::sqrt( std::pow(centroid_x[1] - centroid_x[2], 2) + std::pow(centroid_y[1] - centroid_y[2], 2) );

        //std::cerr << "三个锥桶两两间距: " << dis01 << " " << dis02 << " " << dis12 << "\n" << std::endl;

        if(dis01 < 3.9 && (abs(centroid_dis[0] - centroid_dis[1]) < 2) )
        {
            f_ave_x = centroid_x[0] + centroid_x[1];
            f_ave_y = centroid_y[0] + centroid_y[1];

            a1 = centroid_x[0];
            a2 = centroid_x[1];
            b1 = centroid_y[0];
            b2 = centroid_y[1];
        }
        else if(dis12 < 3.9 && (abs(centroid_dis[1] - centroid_dis[2]) < 2) )
        {
            f_ave_x = centroid_x[1] + centroid_x[2];
            f_ave_y = centroid_y[1] + centroid_y[2];

            a1 = centroid_x[1];
            a2 = centroid_x[2];
            b1 = centroid_y[1];
            b2 = centroid_y[2];
        }
        else if(dis02 < 3.9 && (abs(centroid_dis[0] - centroid_dis[2]) < 2) )
        {
            f_ave_x = centroid_x[0] + centroid_x[2];
            f_ave_y = centroid_y[0] + centroid_y[2];

            a1 = centroid_x[0];
            a2 = centroid_x[2];
            b1 = centroid_y[0];
            b2 = centroid_y[2];
        }
        
        rfans_driver::Point Point_msg;

        Point_msg.a1 = a1;
        Point_msg.a2 = a2;
        Point_msg.b1 = b1;
        Point_msg.b2 = b2;
        Point_msg.a3 = 1000;
        Point_msg.a4 = 1000;
        Point_msg.b3 = 1000;
        Point_msg.b4 = 1000;

        //std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;

        end = clock();														//计时结束
	    //std::cerr << "run time:" << (double)(end-start) / CLOCKS_PER_SEC << "s" << std::endl;


        Point_msg.x1 = f_ave_x / 2;
        Point_msg.y1 = f_ave_y / 2;
        Point_msg.x2 = 1000;
        Point_msg.y2 = 1000;
        pub_point.publish(Point_msg);

        x_1 = Point_msg.x1;
        y_1 = Point_msg.y1;

        std::cerr << std::fixed << "Point_msg : (x1 = " << Point_msg.x1 << ", y1 = " << Point_msg.y1 <<
         ", Point_msg.x2 = " << Point_msg.x2 << ", Point_msg.y2 = " << Point_msg.y2 << "\n" << std::endl;



        std::cerr << std::fixed << "a1 = " << Point_msg.a1 << " , b1 = " << Point_msg.b1 <<
        "\n" << "a2 = " << Point_msg.a2 << " , b2 = " << Point_msg.b2 << 
        "\n" << "a3 = " << Point_msg.a3 << " , b3 = " << Point_msg.b3 <<
        "\n" << "a4 = " << Point_msg.a4 << " , b4 = " << Point_msg.b4 <<  std::endl;




        break;
      }

      default:
      {
        //std::cerr << "The total numeber of cluster :" << cluster_count << "\n" << std::endl;

        std::sort(centroid_dis.begin(),centroid_dis.end()); //从小到大排序
        while(centroid_dis.size() != 4) //只剩四个点
        {
            centroid_dis.pop_back(); //删除最后一个元素，即最远的点
        }

        for(int k = 0; k < centroid_dis.size(); k++)
        { 
            for(int i = 0; i < dis_backup.size(); i++)
            {
                if(centroid_dis[k] == dis_backup[i])
                {
                    //std::cerr << "The distance of cluster:  " << centroid_dis[k] << "m" << std::endl;
                    //std::cerr << "The centroid of cluster: x = " << centroid_x[i] << ", y = " << centroid_y[i] << std::endl;
                    angle = std::atan(centroid_y[i]/centroid_x[i]); //这里暂时是弧度
                    angle = angle*180/PI + 180;
                    //std::cerr << "The angle of centroid:  " << angle << "°" << "\n" << std::endl;
                    use_angle.push_back(angle);
                }
            }
        }

        //int a = centroid_angle.size();
        double f_cones_x[2], f_cones_y[2], f_cones_dis[2], s_cones_x[2], s_cones_y[2], s_cones_dis[2];
        std::sort( use_angle.begin(), use_angle.end() ); //从小到大排序
        for (int i = 0; i < centroid_angle.size(); i++)
        {
            if (use_angle[0] == centroid_angle[i])
            {
                f_cones_x[0] = centroid_x[i];
                f_cones_y[0] = centroid_y[i];
                f_cones_dis[0] = dis_backup[i];
                //std::cerr << "前一对右边锥桶 " << i << " " << centroid_x[i] << " " << centroid_y[i] << " " << dis_backup[i] << std::endl;
            }
            if (use_angle[3] == centroid_angle[i])
            {
                f_cones_x[1] = centroid_x[i];
                f_cones_y[1] = centroid_y[i];
                f_cones_dis[1] = dis_backup[i];
                //std::cerr << "前一对左边锥桶 " << i << " " << centroid_x[i] << " " << centroid_y[i] << " " << dis_backup[i] << std::endl;
            }
            if (use_angle[1] == centroid_angle[i])
            {
                s_cones_x[0] = centroid_x[i];
                s_cones_y[0] = centroid_y[i];
                s_cones_dis[0] = dis_backup[i];
                //std::cerr << "后一对右边锥桶 " << i << " " << centroid_x[i] << " " << centroid_y[i] << " " << dis_backup[i] << std::endl;
            }
            if (use_angle[2] == centroid_angle[i])
            {
                s_cones_x[1] = centroid_x[i];
                s_cones_y[1] = centroid_y[i];
                s_cones_dis[1] = dis_backup[i];
                //std::cerr << "后一对左边锥桶 " << i << " " << centroid_x[i] << " " << centroid_y[i] << " " << dis_backup[i] << std::endl;
            }
        }

        //std::cerr << "\n" << " " << centroid_angle.size() << "\n" << std::endl;

        double f_dis, s_dis;
        f_dis = std::sqrt( std::pow(f_cones_x[0] - f_cones_x[1], 2) + std::pow(f_cones_y[0] - f_cones_y[1], 2) );
        s_dis = std::sqrt( std::pow(s_cones_x[0] - s_cones_x[1], 2) + std::pow(s_cones_y[0] - s_cones_y[1], 2) );
        //std::cerr << "前一对锥桶间距: " << f_dis << " ,后一对锥桶间距: " << s_dis << "\n" << std::endl;

        double f_ave_x = 0,f_ave_y = 0;
        double s_ave_x = 0,s_ave_y = 0;

        if(f_dis > 3.9 && s_dis > 3.9)//两对锥桶全部匹配错误
        {
            //std::cerr << "f_dis > 3.9 && s_dis > 3.9" << "\n" << std::endl;
            //if(f_dis > 3.9 && s_dis > 3.9)

            if(f_cones_dis[1] > s_cones_dis[1])//前后两对锥桶左边颠倒
            {
                //std::cerr << "f_cones_dis[1] > s_cones_dis[1]" << "\n" << std::endl;
                double x, y, dis;
                x = f_cones_x[1];
                y = f_cones_y[1];
                dis = f_cones_dis[1];
                f_cones_x[1] = s_cones_x[1];
                s_cones_x[1] = x;
                f_cones_y[1] = s_cones_y[1];
                s_cones_y[1] = y;
                f_cones_dis[1] = s_cones_dis[1];
                s_cones_dis[1] = dis;

                a1 = f_cones_x[0];
                a2 = f_cones_x[1];
                b1 = f_cones_y[0];
                b2 = f_cones_y[1];
                a3 = s_cones_x[0];
                a4 = s_cones_x[1];
                b3 = s_cones_y[0];
                b4 = s_cones_y[1];

                f_ave_x = f_cones_x[0] + f_cones_x[1];
                f_ave_y = f_cones_y[0] + f_cones_y[1];
                s_ave_x = s_cones_x[0] + s_cones_x[1];
                s_ave_y = s_cones_y[0] + s_cones_y[1];
                //std::cerr << " " << f_cones_x[0] << " " << f_cones_y[0] << " " << std::endl;
                //std::cerr << " " << f_cones_x[1] << " " << f_cones_y[1] << " " << std::endl;
                //std::cerr << " " << s_cones_x[0] << " " << s_cones_y[0] << " " << std::endl;
                //std::cerr << " " << s_cones_x[1] << " " << s_cones_y[1] << " " << std::endl;
                //std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
                //std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
            }
            if(f_cones_dis[0] > s_cones_dis[0])//前后两对锥桶右边颠倒
            {
                //std::cerr << "f_cones_dis[0] > s_cones_dis[0]" << "\n" << std::endl;
                double x, y, dis;
                x = f_cones_x[0];
                y = f_cones_y[0];
                dis = f_cones_dis[0];
                f_cones_x[0] = s_cones_x[0];
                s_cones_x[0] = x;
                f_cones_y[0] = s_cones_y[0];
                s_cones_y[0] = y;
                f_cones_dis[0] = s_cones_dis[0];
                s_cones_dis[0] = dis;

                a1 = f_cones_x[0];
                a2 = f_cones_x[1];
                b1 = f_cones_y[0];
                b2 = f_cones_y[1];
                a3 = s_cones_x[0];
                a4 = s_cones_x[1];
                b3 = s_cones_y[0];
                b4 = s_cones_y[1];

                f_ave_x = f_cones_x[0] + f_cones_x[1];
                f_ave_y = f_cones_y[0] + f_cones_y[1];
                s_ave_x = s_cones_x[0] + s_cones_x[1];
                s_ave_y = s_cones_y[0] + s_cones_y[1];
                //std::cerr << " " << f_cones_x[0] << " " << f_cones_y[0] << " " << std::endl;
                //std::cerr << " " << f_cones_x[1] << " " << f_cones_y[1] << " " << std::endl;
                //std::cerr << " " << s_cones_x[0] << " " << s_cones_y[0] << " " << std::endl;
                //std::cerr << " " << s_cones_x[1] << " " << s_cones_y[1] << " " << std::endl;
                //std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
                //std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
            }
            else 
            {
                if(f_cones_dis[0] = dis_backup[0])//1-逆时针转弯时左侧只有5m位置的锥桶，右侧三个锥桶；2-右侧0m，5m位置锥桶，左侧5m，10m位置锥桶（逆时针转弯时左侧锥桶角度匹配正确）
                {
                    //std::cerr << "f_cones_dis[0] < 2" << "\n" << std::endl;
                    f_ave_x = f_cones_x[1] + s_cones_x[0];
                    f_ave_y = f_cones_y[1] + s_cones_y[0];
                    s_ave_x = 2000;
                    s_ave_y = 2000;

                    a1 = f_cones_x[1];
                    a2 = s_cones_x[0];
                    b1 = f_cones_y[1];
                    b2 = s_cones_y[0];
                    a3 = 1000;
                    a4 = 1000;
                    b3 = 1000;
                    b4 = 1000;

                    //std::cerr << " " << f_cones_x[1] << " " << f_cones_y[1] << " " << std::endl;
                    //std::cerr << " " << s_cones_x[0] << " " << s_cones_y[0] << " " << std::endl;

                    //std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
                    //std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
                }
                else if(f_cones_dis[1] = dis_backup[0])//1-顺时针转弯时右侧只有5m位置的锥桶，左侧三个锥桶；2-左侧0m，5m位置锥桶，右侧5m，10m位置锥桶（顺时针转弯时右侧锥桶角度匹配正确）
                {
                    //std::cerr << "f_cones_dis[1] < 2" << "\n" << std::endl;
                    f_ave_x = f_cones_x[0] + s_cones_x[1];
                    f_ave_y = f_cones_y[0] + s_cones_y[1];
                    s_ave_x = 2000;
                    s_ave_y = 2000;

                    a1 = f_cones_x[0];
                    a2 = s_cones_x[1];
                    b1 = f_cones_y[0];
                    b2 = s_cones_y[1];
                    a3 = 1000;
                    a4 = 1000;
                    b3 = 1000;
                    b4 = 1000;

                    //std::cerr << " " << f_cones_x[1] << " " << f_cones_y[1] << " " << std::endl;
                    //std::cerr << " " << s_cones_x[0] << " " << s_cones_y[0] << " " << std::endl;

                    //std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
                    //std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
                }
                else 
                {
                    //std::cerr << "f_dis > 3.9 && s_dis > 3.9 未处理" << "\n" << std::endl;

                    a1 = 1000;
                    a2 = 1000;
                    b1 = 1000;
                    b2 = 1000;
                    a3 = 1000;
                    a4 = 1000;
                    b3 = 1000;
                    b4 = 1000;

                    f_ave_x = 2000;
                    f_ave_y = 2000;
                    s_ave_x = 2000;
                    s_ave_y = 2000;

                    /*if(f_cones_dis[1] < 2)
                    {
                        std::cerr << "f_cones_dis[1] < 2" << "\n" << std::endl;
                        int i;
                        i = (s_cones_dis[0] < s_cones_dis[1]) ? 0 : 1;
                        f_ave_x = f_cones_x[0] + s_cones_x[i];
                        f_ave_y = f_cones_y[0] + s_cones_y[i];
                        s_ave_x = 2000;
                        s_ave_y = 2000;
                        std::cerr << " " << f_cones_x[1] << " " << f_cones_y[1] << " " << std::endl;
                        std::cerr << " " << s_cones_x[0] << " " << s_cones_y[0] << " " << std::endl;

                        std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
                        std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
                    }
                    */
                }
            }
        }
        else if(f_dis > 3.9 && s_dis < 3.9)//第一对锥桶匹配错误，第二对锥桶可能.匹配错误
        {
            //std::cerr << "f_dis > 3.9 && s_dis < 3.9" << "\n" << std::endl;
            double f_centroid_dis, s_centroid_dis;
            f_centroid_dis = std::sqrt( std::pow((f_cones_x[0]+f_cones_x[1]) / 2, 2) + std::pow((f_cones_y[0]+f_cones_y[1]) / 2, 2) );
            s_centroid_dis = std::sqrt( std::pow((s_cones_x[0]+s_cones_x[1]) / 2, 2) + std::pow((s_cones_y[0]+s_cones_y[1]) / 2, 2) );
            //double dis_f0si;
            //dis_f0si = std::sqrt( std::pow(f_cones_x[0] - s_cones_x[1], 2) + std::pow(f_cones_y[0] - s_cones_y[1], 2) );
            //std::cerr << "dis_f0si = " << dis_f0si << "\n" << std::endl;
            if(abs(f_centroid_dis - s_centroid_dis) < 1.5)//1-逆时针转弯时右侧0m，5m两个锥桶，左侧5m，10m两个锥桶（左侧锥桶角度匹配错误）
                                        //2-顺时针转弯时左侧0m，5m两个锥桶，右侧5m，10m两个锥桶（右侧锥桶角度匹配错误）                                                            
            {
                //std::cerr << "(f_centroid_dis - s_centroid_dis) < 1.5" << "\n" << std::endl;

                a1 = s_cones_x[0];
                a2 = s_cones_x[1];
                b1 = s_cones_y[0];
                b2 = s_cones_y[1];
                a3 = 1000;
                a4 = 1000;
                b3 = 1000;
                b4 = 1000;

                f_ave_x = s_cones_x[0] + s_cones_x[1];
                f_ave_y = s_cones_y[0] + s_cones_y[1];
                s_ave_x = 2000;
                s_ave_y = 2000;

                //std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
                //std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
            }
            else//if((f_dis - s_dis) > 1.5)，即 1-逆时针转弯时右侧只有5m位置的锥桶，左侧三个锥桶；2-顺时针转弯时右侧三个锥桶，左侧只有5m位置的锥桶
            {
                //std::cerr << "(f_centroid_dis - s_centroid_dis) >= 1.5" << "\n" << std::endl;
                int i,j;
                i = (f_cones_dis[0] > f_cones_dis[1]) ? 0 : 1;
                j = (s_cones_dis[0] < s_cones_dis[1]) ? 0 : 1;

                a1 = f_cones_x[i];
                a2 = s_cones_x[j];
                b1 = f_cones_y[i];
                b2 = s_cones_y[j];
                a3 = 1000;
                a4 = 1000;
                b3 = 1000;
                b4 = 1000;

                f_ave_x = f_cones_x[i] + s_cones_x[j];
                f_ave_y = f_cones_y[i] + s_cones_y[j];
                s_ave_x = 2000;
                s_ave_y = 2000;

                //std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
                //std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
            }
        }
        else//第一对锥桶匹配正确
        {
            //std::cerr << "normal" << "\n" << std::endl;

            a1 = f_cones_x[0];
            a2 = f_cones_x[1];
            b1 = f_cones_y[0];
            b2 = f_cones_y[1];
            a3 = s_cones_x[0];
            a4 = s_cones_x[1];
            b3 = s_cones_y[0];
            b4 = s_cones_y[1];

            f_ave_x = f_cones_x[0] + f_cones_x[1];
            f_ave_y = f_cones_y[0] + f_cones_y[1];
            s_ave_x = s_cones_x[0] + s_cones_x[1];
            s_ave_y = s_cones_y[0] + s_cones_y[1];
            //std::cerr << " " << f_cones_x[0] << " " << f_cones_y[0] << " " << std::endl;
            //std::cerr << " " << f_cones_x[1] << " " << f_cones_y[1] << " " << std::endl;
            //std::cerr << " " << s_cones_x[0] << " " << s_cones_y[0] << " " << std::endl;
            //std::cerr << " " << s_cones_x[1] << " " << s_cones_y[1] << " " << std::endl;
            //std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
            //std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
        }



/*        if ( f_dis > 4 || s_dis > 4) 
        {
            f_ave_x = f_cones_x[1] + s_cones_x[0];
            f_ave_y = f_cones_y[1] + s_cones_y[0];
            s_ave_x = 2000;
            s_ave_y = 2000;
            std::cerr << " " << f_cones_x[1] << " " << f_cones_y[1] << " " << std::endl;
            std::cerr << " " << s_cones_x[0] << " " << s_cones_y[0] << " " << std::endl;

            std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
            std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
        }
        else
        {
            f_ave_x = f_cones_x[0] + f_cones_x[1];
            f_ave_y = f_cones_y[0] + f_cones_y[1];
            s_ave_x = s_cones_x[0] + s_cones_x[1];
            s_ave_y = s_cones_y[0] + s_cones_y[1];
            std::cerr << " " << f_cones_x[0] << " " << f_cones_y[0] << " " << std::endl;
            std::cerr << " " << f_cones_x[1] << " " << f_cones_y[1] << " " << std::endl;
            std::cerr << " " << s_cones_x[0] << " " << s_cones_y[0] << " " << std::endl;
            std::cerr << " " << s_cones_x[1] << " " << s_cones_y[1] << " " << std::endl;
            std::cerr << "The first of centroid point: (x = " << f_ave_x / 2 << ", y = " << f_ave_y / 2 << ")" << std::endl;
            std::cerr << "The second of centroid point: (x = " << s_ave_x / 2 << ", y = " << s_ave_y / 2 << ")" << "\n" << std::endl;
        }
*/
        end = clock();														//计时结束
        //std::cerr << "run time:" << (double)(end-start) / CLOCKS_PER_SEC << "s" << std::endl;

        rfans_driver::Point Point_msg;
        double x1, x2, y1, y2;
        //conversion(gps_pionts, f_ave_x / 2, f_ave_y / 2, &x1, &y1, 1.5);
        //conversion(gps_pionts, s_ave_x / 2, s_ave_y / 2, &x2, &y2, 1.5);
        
        x1 = f_ave_x / 2;
        y1 = f_ave_y / 2;
        x2 = s_ave_x / 2;
        y2 = s_ave_y / 2;
        
        /*xs1 = x1;
        ys1 = y1;
        xs2 = x2;
        ys2 = y2;   
        */    
        Point_msg.a1 = a1;
        Point_msg.a2 = a2;
        Point_msg.a3 = a3;
        Point_msg.a4 = a4;
        Point_msg.b1 = b1;
        Point_msg.b2 = b2;
        Point_msg.b3 = b3;
        Point_msg.b4 = b4;

        Point_msg.x1 = x1;
        Point_msg.y1 = y1;
        Point_msg.x2 = x2;
        Point_msg.y2 = y2;
        pub_point.publish(Point_msg);

        x_1 = Point_msg.x1;
        y_1 = Point_msg.y1;

        std::cerr << std::fixed << "centroid point 1: x = " << Point_msg.x1 << " , y = " << Point_msg.y1 <<
        "\n" << "centroid point 2: x = " << Point_msg.x2 << " , y = " << Point_msg.y2 << "\n" << std::endl;


 std::cerr << std::fixed << "a1 = " << Point_msg.a1 << " , b1 = " << Point_msg.b1 <<
        "\n" << "a2 = " << Point_msg.a2 << " , b2 = " << Point_msg.b2 << 
        "\n" << "a3 = " << Point_msg.a3 << " , b3 = " << Point_msg.b3 <<
        "\n" << "a4 = " << Point_msg.a4 << " , b4 = " << Point_msg.b4 <<  std::endl;

        break;
      }
    }
    
    //保存聚类点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        cloud_output->points.push_back(cloud->points[*pit]);
        cloud_output->width = cloud_output->points.size();
        cloud_output->height = 1;
        cloud_output->is_dense = true;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid_point(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_centroid_point=*cloud_output;
    cloud_centroid_point->width = 150 + cloud_output->points.size ();
    cloud_centroid_point->height = 1;
    cloud_centroid_point->is_dense = true;
    cloud_centroid_point->points.resize (cloud_centroid_point->width * cloud_centroid_point->height);
    //for(size_t i = 0; i < cloud_centroid_point->points.size (); )
    //{
    if(x_1 != 1000)
    {
        //std::cerr << "开始往cloud_centroid_point写入点云" <<std::endl;
        for(int i = cloud_output->points.size (), k = 0; k < 10 ; k++)//cloud_output->points.size ()
        {
            for(int l = 0; l < 10; l++)
            {
                //std::cerr << "往cloud_centroid_point写入第" << i << "个点云" <<std::endl;
                cloud_centroid_point->points[i].x = x_1 - 0.15 + 0.03*k;
                cloud_centroid_point->points[i].y = y_1 - 0.15 + 0.03*l;
                cloud_centroid_point->points[i].z = 0;
                i++;
            }
        }
    //}

    /*for(int i = 0; i <  cloud_centroid_point->points.size (); i++)
    {
         std::cerr << "point_" << i << ": x =  " << cloud_centroid_point->points[i].x << ", y = " << cloud_centroid_point->points[i].y
          << " , z = " << cloud_centroid_point->points[i].z << "\n" << std::endl;
    }
    */
        //std::cerr << "往cloud_centroid_point写入点云 完成" <<std::endl;
        sensor_msgs::PointCloud2 cloud_output_centroid;
        pcl::toROSMsg(*cloud_centroid_point, cloud_output_centroid);
        cloud_output_centroid.header.frame_id = "pandar";
        pub_cloud_output_centroid.publish(cloud_output_centroid);

        //std::cerr << "Topic cloud_output_point published" << "\n" <<std::endl;
    }


  //ros::Rate loop_rate(5);
  //loop_rate.sleep();
    
}
private:
  ros::NodeHandle n;
  ros::Publisher pub_point;
  ros::Publisher pub_cloud_output_centroid;

  ros::Subscriber sub_final_PointCloud2;

};
//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish1");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish1 SAPObject;

  ros::spin();

  return 0;
}