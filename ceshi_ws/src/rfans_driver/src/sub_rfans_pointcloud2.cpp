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

//全部点云提取锥桶

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()  //构造函数
  {
    //Topic you want to publish  创建publish
    pub_cloud_filtered = n.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 1);//直通滤波，切割出处理区域
    pub_cloud_re = n.advertise<sensor_msgs::PointCloud2> ("cloud_revolution", 1);//旋转后的点云
    pub_cloud_other = n.advertise<sensor_msgs::PointCloud2> ("cloud_other", 1);//去除地面(SAC_RANSAC)
    pub_cloud_output_projected = n.advertise<sensor_msgs::PointCloud2> ("cloud_projected", 1);//投影到X-Y平面

    pub_cloud_output_sor = n.advertise<sensor_msgs::PointCloud2> ("cloud_sor", 1);//统计滤波（去除离群点）
    pub_cloud_output_ror = n.advertise<sensor_msgs::PointCloud2> ("cloud_ror", 1);//半径滤波（去除离群点）
  
    pub_cloud_output_final = n.advertise<sensor_msgs::PointCloud2> ("cloud_final", 1);//发布给计算中心点节点的点云

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
    pass_y.setFilterLimits(-10, 10);
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
 
/*
    //统计滤波(去除离群点)
    // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1
    // 这意味着如果一个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_sor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; //创建滤波器对象
    sor.setInputCloud(cloud); //设置待滤波的点云
    sor.setMeanK(50); //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(0.01);     //设置判断是否为离群点的阀值
    sor.filter(*cloud_filtered_sor); //存储
    *cloud = *cloud_filtered_sor;

    sensor_msgs::PointCloud2 cloud_output_sor;
    pcl::toROSMsg(*cloud_filtered_sor, cloud_output_sor);
    cloud_output_sor.header.frame_id = "pandar";
    pub_cloud_output_sor.publish(cloud_output_sor); 
*/

    //半径滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ror(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror; //创建滤波器对象
    ror.setInputCloud(cloud);                     //设置待滤波的点云
    ror.setRadiusSearch(0.05);                     //设置查询半径
    ror.setMinNeighborsInRadius(5);  //设置查询半径中点的个数
    ror.filter(*cloud_filtered_ror); //存储点云
    *cloud = *cloud_filtered_ror;

    sensor_msgs::PointCloud2 cloud_output_ror;
    pcl::toROSMsg(*cloud_filtered_ror, cloud_output_ror);
    cloud_output_ror.header.frame_id = "pandar";
    pub_cloud_output_ror.publish(cloud_output_ror);

    sensor_msgs::PointCloud2 cloud_output_final;
    pcl::toROSMsg(*cloud, cloud_output_final);
    cloud_output_final.header.frame_id = "pandar";
    pub_cloud_output_final.publish(cloud_output_final);
    
    std::cerr << "OK " ;
  }

private:
  ros::NodeHandle n;
  ros::Publisher pub_cloud_filtered;
  ros::Publisher pub_cloud_re;
  ros::Publisher pub_cloud_other;
  ros::Publisher pub_cloud_output_projected;
  ros::Publisher pub_cloud_output_sor;
  ros::Publisher pub_cloud_output_ror;
  ros::Publisher pub_cloud_output_final;

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