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

//全部点云提取锥桶,不配对
const double PI = 3.141592653589;
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()  //构造函数
  {
    //Topic you want to publish  创建publish
    pub_cloud_filtered = n.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 1);//直通滤波，切割出处理区域
    pub_cloud_re = n.advertise<sensor_msgs::PointCloud2> ("cloud_revolution", 1);//旋转后的点云
    pub_cloud_other = n.advertise<sensor_msgs::PointCloud2> ("cloud_other", 1);//去除地面(SAC_RANSAC)
    //pub_cloud_output_projected = n.advertise<sensor_msgs::PointCloud2> ("cloud_projected", 1);//投影到X-Y平面
    //pub_cloud_output_final = n.advertise<sensor_msgs::PointCloud2> ("cloud_final", 1);//发布给计算中心点节点的点云
    pub_cloud_output_small = n.advertise<sensor_msgs::PointCloud2> ("cloud_small", 1);
    //Topic you want to subscribe 创建subscribe
    sub_PointCloud2 = n.subscribe("/pandar_points", 1, &SubscribeAndPublish::cloud_callback, this);//雷达原始点云
  }
 
  //得出两个向量之间的旋转矩阵（切割的大地平面法向量和雷达的z轴不一定重合，作用是转换为重合），网上代码
  Eigen::Matrix4f CreateRotationMatrix(Eigen::Vector3f angle_before,Eigen::Vector3f angle_after)
   {
    angle_before.normalize();
    angle_after.normalize();
    float angle = acos(angle_before.dot(angle_after));
    Eigen::Vector3f p_rotate = angle_before.cross(angle_after);
    p_rotate.normalize();
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle)); //这里跟公式比多了一个括号，但是看实验结果它是对的。
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));
    return rotationMatrix;
   }

  //过滤较近和较远的点，网上代码
  void remove_close_pt(double min_distance, double max_distance,const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr out) 
  {
    pcl::ExtractIndices<pcl::PointXYZ> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    //#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++) 
    {
      double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);
      if (distance < min_distance || distance > max_distance) 
      {
        indices.indices.push_back(i);
      }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); // ture to remove the indices
    cliper.filter(*out);
  }

  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &input) 
  {
    clock_t start, end;
    start = clock();

    // 将msg点云格式为sensor_msgs/PointCloud2 格式转为
    // pcl/PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    //切割z轴区域
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-0.4, 1.5);
    pass_z.setFilterLimitsNegative(false); //设置过滤器限制负//设置保留范围内false
    pass_z.filter(*cloud_source_filtered_z);
    *cloud = *cloud_source_filtered_z;

    //过滤较近和较远距离的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZ>);
    remove_close_pt(0.1, 15, cloud, remove_close);
    *cloud = *remove_close;

    //切割出处理区域
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0.1, 15);
    pass_x.setFilterLimitsNegative(false); //设置过滤器限制负//设置保留范围内false
    pass_x.filter(*cloud_source_filtered_x);
    *cloud = *cloud_source_filtered_x;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-4, 4);
    pass_y.setFilterLimitsNegative(false); //设置过滤器限制负//设置保留范围内false
    pass_y.filter(*cloud_source_filtered_y);
    *cloud = *cloud_source_filtered_y;

    sensor_msgs::PointCloud2 cloud_filtered;
    pcl::toROSMsg(*cloud_source_filtered_y, cloud_filtered);
    cloud_filtered.header.frame_id = "pandar";
    pub_cloud_filtered.publish(cloud_filtered);


    //切割出提取平面法向量的点云范围   (既切割出.地面，为了下一步旋转平面)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x_plane;
    pass_x_plane.setInputCloud(cloud);
    pass_x_plane.setFilterFieldName("x");
    pass_x_plane.setFilterLimits(0.3, 6);
    pass_x_plane.setFilterLimitsNegative(false); //设置过滤器限制负//设置保留范围内false
    pass_x_plane.filter(*cloud_filtered_x_plane);
    *cloud_y_plane = *cloud_filtered_x_plane;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_seg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y_plane;
    pass_y_plane.setInputCloud(cloud_y_plane);
    pass_y_plane.setFilterFieldName("y");
    pass_y_plane.setFilterLimits(-1, 1);
    pass_y_plane.setFilterLimitsNegative(false); //设置过滤器限制负//设置保留范围内false
    pass_y_plane.filter(*cloud_filtered_y_plane);
    *cloud_plane_seg = *cloud_filtered_y_plane;

    //旋转平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_revolution(new pcl::PointCloud<pcl::PointXYZ>);
    //创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    //创建分割时所需要的模型系数对象coefficients及存储内点的点索引集合对象inliers
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_rp(new pcl::ModelCoefficients);
    //可选择配置，设置模型系数需要优化
    plane_seg.setOptimizeCoefficients(true);
    //必须配置，设置分割的模型类型、所用的随机参数估计方法、距离阈值、输入点云
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.1);
    plane_seg.setInputCloud(cloud_plane_seg);
    //引发分割实现，并存储分割结果到点集合inliers，并储存平面模型系数coefficients
    plane_seg.segment(*inliers,*coefficients_rp); //得到平面系数，进而得到平面法向量
    if (inliers->indices.size() == 0) 
    {
      PCL_ERROR("Could not estimate a planar model for the given dataset.");
      *cloud = *cloud_revolution;
      //*cloud_z = *cloud_source_filtered_y;
    } 
    else 
    {
      //打印出估计的平面模型参数
      /*
      std::cerr << "Model coefficients: " << coefficients_rp->values[0] << " "
                << coefficients_rp->values[1] << " "
                << coefficients_rp->values[2] << " "
                << coefficients_rp->values[3] << std::endl;
      */
      Eigen::Vector3f ground_xyz(coefficients_rp->values[0],
                                 coefficients_rp->values[1],
                                 coefficients_rp->values[2]);
      Eigen::Vector3f ridar_xyz(0.0f, 0.0f, 1.0f);
      Eigen::Matrix4f rotation = CreateRotationMatrix(ground_xyz, ridar_xyz);
      pcl::transformPointCloud(*cloud, *cloud_revolution,rotation); //得到旋转之后的点云
      *cloud = *cloud_revolution;
    
    sensor_msgs::PointCloud2 cloud_re;
    pcl::toROSMsg(*cloud_revolution, cloud_re);
    cloud_re.header.frame_id = "pandar";
    pub_cloud_re.publish(cloud_re);

    }

    //去除地面
    //设置分割类型,这里使用ransac分割平面
    pcl::SACSegmentation<pcl::PointXYZ> seg; //创建分割对象
    seg.setOptimizeCoefficients(true); //设置对估计模型参数进行优化处理
    seg.setModelType(pcl::SACMODEL_PLANE); //设置分割模型类别
    // seg.setNormalDistanceWeight (0.1);
    // seg.setInputNormals (cloud_normals);
    // //要是使用法线估计pcl::SACMODEL_NORMAL_PLANE就要用上面两句话 normal
    seg.setMethodType(pcl::SAC_RANSAC); //设置用哪个随机参数估计方法
    seg.setMaxIterations(150);          //设置最大迭代次数
    seg.setDistanceThreshold(0.12); //判断是否为模型内点的距离阀值
    seg.setInputCloud(cloud);

    //分割平面，并提取数据(去除地面的点，会影响识别锥桶)
    pcl::PointIndices::Ptr inliers1(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::ExtractIndices<pcl::PointXYZ> extract; //创建点云提取对象
    // pcl::ExtractIndices<pcl::Normal> extract_normals;
    // //创建法线提取对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

    seg.segment(*inliers1, *coefficients); //分割
    // std::cerr << "Plane coefficients: " << *coefficients << std::endl;
    extract.setInputCloud(cloud); //设置输入点云数据
    extract.setIndices(inliers1); //设置指定点云数据
    extract.setNegative(true); //指定点外的点'云，即除平面外的物体
    extract.filter(*cloud_other); //输出其他点，去除平面的点云
    extract.setNegative(false);
    extract.filter(*cloud_plane); //输出指定点，平面
    // std::cerr << "segment cloud_plane succeed" << std::endl;
    *cloud = *cloud_other;

    sensor_msgs::PointCloud2 cloud_output_other;
    pcl::toROSMsg(*cloud_other, cloud_output_other);
    cloud_output_other.header.frame_id = "pandar";
    pub_cloud_other.publish(cloud_output_other);


/*
    //使用参数化模型投影点云
    //填充ModelCoefficients的值,使用ax+by+cz+d=0平面模型，其中 a=b=d=0,c=1
    //也就是X-Y平面
    //定义模型系数对象，并填充对应的数据
    pcl::ModelCoefficients::Ptr coefficients_xy(new pcl::ModelCoefficients());
    coefficients_xy->values.resize(4);
    coefficients_xy->values[0] = 0;
    coefficients_xy->values[1] = 0;
    coefficients_xy->values[2] = 1.0;
    coefficients_xy->values[3] = 0;

    //创建ProjectInliers对象，使用ModelCoefficients作为投影对象的模型参数
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj; //创建投影滤波对象
    proj.setModelType(pcl::SACMODEL_PLANE);  //设置对象对应的投影模型
    proj.setInputCloud(cloud);               //设置输入点云
    proj.setModelCoefficients(coefficients_xy); //设置模型对应的系数
    proj.filter(*cloud_projected);              //投影结果存储
    *cloud = *cloud_projected; //这里得到的cloud中存储的点的z坐标均为0为，即投影到X—Y平面

    sensor_msgs::PointCloud2 cloud_output_projected;
    pcl::toROSMsg(*cloud_projected, cloud_output_projected);
    cloud_output_projected.header.frame_id = "pandar";
    pub_cloud_output_projected.publish(cloud_output_projected);
 */
    
/*
    sensor_msgs::PointCloud2 cloud_output_final;
    pcl::toROSMsg(*cloud, cloud_output_final);
    cloud_output_final.header.frame_id = "pandar";
    pub_cloud_output_final.publish(cloud_output_final);
*/   


    //欧式聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);;
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;				       	//创建点云索引向量，用于存储实际的点云信息
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;	                //欧式聚类对象
    ec.setClusterTolerance (0.3);								    	//设置近邻搜索的搜索半径，这里单位为m
    ec.setMinClusterSize (15);								            //设置一个聚类需要的最少的点数目为2000
    ec.setMaxClusterSize (1000);								      	//设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree);								        	//设置点云的搜索机制
    ec.setInputCloud (cloud);					                        //输入需要聚类的点云指针地址
    ec.extract (cluster_indices);								      	//得到2000-25000点的聚类

    std::cerr << "聚类数量: " << cluster_indices.size() << "\n" << std::endl;

    //为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中
    //迭代访问点云索引cluster_indices,直到分割出所有聚类
    int cluster_count = 0, cluster_count_small = 0;
    int j = 0;
    double angle;       //angle为角度制
    std::vector<double> centroid_x, centroid_y, centroid_dis, dis_backup, centroid_angle, use_angle, use_y, right_x, right_y, left_x, left_y;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_small(new pcl::PointCloud<pcl::PointXYZ>);
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
/*
//z轴限制 过滤锥桶
        if((max.z - min.z < 0.35) && (max.z - min.z > 0.05))//提取小锥桶,过滤其他物体，但会出现第三对锥桶闪的情况
         {
            if(max.x - min.x > 0.30 || max.y -min.y > 0.30 )
            {}
            else
            {
            cluster_count_small++;

            std::cerr << "Cluster " << cluster_count_small << " has " << cloud_cluster->points.size () << " data points." << std::endl;
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

            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster_small->points.push_back(cloud->points[*pit]);
            cloud_cluster_small->width = cloud_cluster_small->points.size();
            cloud_cluster_small->height = 1;
            cloud_cluster_small->is_dense = true;

            j++;  
            }            
         }
    }
*/     


//z轴不限制
            std::cerr << "Cluster " << cluster_count << " has " << cloud_cluster->points.size () << " data points." << std::endl;
            centroid_x.push_back( sum_x / cloud_cluster->points.size() );
            centroid_y.push_back( sum_y / cloud_cluster->points.size() );
            use_y.push_back( sum_y / cloud_cluster->points.size() );//用来对y排序
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

        rfans_driver::Point Point_msg;
        double a1, a2, a3, a4, b1, b2, b3, b4, d1;
        std::sort(use_y.begin(),use_y.end()); //从小到大排序
           for (int i = 0; i <  centroid_y.size(); i++)
        {
            if (use_y[0] == centroid_y[i])
            {
                a1 = centroid_x[i];
                b1 = centroid_y[i];
            }
        }

           for (int i = 0; i <  centroid_y.size(); i++)
        {
            if (std::sqrt( std::pow((a1-centroid_x[i]), 2) + std::pow((b1-centroid_y[i]), 2) ) == 0)
            {                
                right_x.push_back(centroid_x[i]);
                right_y.push_back(centroid_y[i]);
                //std::cerr << "第一个右边锥桶 " << i << " x=" << centroid_x[i] << " y=" << centroid_y[i] << std::endl;
            } 
            else if (4.9 < std::sqrt( std::pow((a1-centroid_x[i]), 2) + std::pow((b1-centroid_y[i]), 2) ) && std::sqrt( std::pow((a1-centroid_x[i]), 2) + std::pow((b1-centroid_y[i]), 2) ) <5.1)  
            {
                right_x.push_back(centroid_x[i]);
                right_y.push_back(centroid_y[i]);
                //std::cerr << "第二个右边锥桶 " << i << " x=" << centroid_x[i] << " y=" << centroid_y[i] << std::endl;
            }
            else if (9.9 < std::sqrt( std::pow((a1-centroid_x[i]), 2) + std::pow((b1-centroid_y[i]), 2) ) && std::sqrt( std::pow((a1-centroid_x[i]), 2) + std::pow((b1-centroid_y[i]), 2) ) < 10.1)
            {
                right_x.push_back(centroid_x[i]);
                right_y.push_back(centroid_y[i]);
                //std::cerr << "第三个右边锥桶 " << i << " x=" << centroid_x[i] << " y=" << centroid_y[i] << std::endl;
            }
            else if (14.9 < std::sqrt( std::pow((a1-centroid_x[i]), 2) + std::pow((b1-centroid_y[i]), 2) ) && std::sqrt( std::pow((a1-centroid_x[i]), 2) + std::pow((b1-centroid_y[i]), 2) ) < 15.1)
            {                
                right_x.push_back(centroid_x[i]);
                right_y.push_back(centroid_y[i]);
                //std::cerr << "第四个右边锥桶 " << i << " x=" << centroid_x[i] << " y=" << centroid_y[i] << std::endl;
            }
            else
            {
                left_x.push_back(centroid_x[i]);
                left_y.push_back(centroid_y[i]);
            }           
        }

        for (int i = 0; i <  right_x.size(); i++)
        {
            std::cerr << " 右边锥桶 " << i << " x=" << right_x[i] << " y=" << right_y[i] << std::endl;
        }
        for (int i = 0; i <  left_x.size(); i++)
        {
            std::cerr << " 左边锥桶 " << i << " x=" << left_x[i] << " y=" << left_y[i] << std::endl;
        }

/*

    *cloud=*cloud_cluster_small;
    std::cerr << "小锥桶数量: " << cluster_count_small << "\n" << std::endl;

    sensor_msgs::PointCloud2 cloud_output_small;
    pcl::toROSMsg(*cloud_cluster_small, cloud_output_small);
    cloud_output_small.header.frame_id = "pandar";
    pub_cloud_output_small.publish(cloud_output_small);
*/



  }

private:
  ros::NodeHandle n;
  ros::Publisher pub_cloud_filtered;
  ros::Publisher pub_cloud_re;
  ros::Publisher pub_cloud_other;
  //ros::Publisher pub_cloud_output_projected;
  //ros::Publisher pub_cloud_output_final;
  ros::Publisher pub_cloud_output_small;

  ros::Subscriber sub_PointCloud2;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}


