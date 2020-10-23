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
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <sensor_msgs/PointCloud2.h>
#include <rfans_driver/point_zx.h>
#include <rfans_driver/gps_data.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
//#include <time.h>

//全部点云提取锥桶,不配对,最多发布6个锥桶，适用于直线。

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
    pub_cloud_output_small = n.advertise<sensor_msgs::PointCloud2> ("cloud_small", 1);//发布给摄像头的锥桶点云
    //pub_cloud_output_final = n.advertise<sensor_msgs::PointCloud2> ("cloud_final", 1);//发布给计算中心点节点的点云
	pub_cloud_greedy = n.advertise<sensor_msgs::PointCloud2>("cloud_greedy", 1);//发布重建后的点云
    pub_point = n.advertise<rfans_driver::point_zx>("Point", 1);//锥桶中心点坐标 发给决策
    //Topic you want to subscribe 创建subscribe

    sub_PointCloud2 = n.subscribe("/rslidar_points", 1, &SubscribeAndPublish::cloud_callback, this);//雷达原始点云
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
    double x_left,x_right,y_left,y_right,point_min,point_max;
    n.getParam("x_left", x_left);
    n.getParam("x_right", x_right);
    n.getParam("y_left", y_left);
    n.getParam("y_right", y_right);
    n.getParam("point_min", point_min);
	n.getParam("point_max", point_max);
    
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
    pass_x.setFilterLimits(x_left, x_right);
    pass_x.setFilterLimitsNegative(false); //设置过滤器限制负//设置保留范围内false
    pass_x.filter(*cloud_source_filtered_x);
    *cloud = *cloud_source_filtered_x;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_left, y_right);
    pass_y.setFilterLimitsNegative(false); //设置过滤器限制负//设置保留范围内false
    pass_y.filter(*cloud_source_filtered_y);
    *cloud = *cloud_source_filtered_y;

    sensor_msgs::PointCloud2 cloud_filtered;
    pcl::toROSMsg(*cloud_source_filtered_y, cloud_filtered);
    cloud_filtered.header.frame_id = "rslidar";
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
    cloud_re.header.frame_id = "rslidar";
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
    seg.setDistanceThreshold(0.1); //判断是否为模型内点的距离阀值
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
    cloud_output_other.header.frame_id = "rslidar";
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
    cloud_output_projected.header.frame_id = "rslidar";
    pub_cloud_output_projected.publish(cloud_output_projected);
*/
    
/*
    sensor_msgs::PointCloud2 cloud_output_final;
    pcl::toROSMsg(*cloud, cloud_output_final);
    cloud_output_final.header.frame_id = "rslidar";
    pub_cloud_output_final.publish(cloud_output_final);
*/ 

    //点云重建
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针
	tree2->setInputCloud(cloud);   ///用cloud构建tree对象
	n.setInputCloud(cloud);
	n.setSearchMethod(tree2);
	n.setKSearch(20);
	n.compute(*normals);       ////估计法线存储到其中
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);    //连接字段
	//* cloud_with_normals = cloud + normals

	//定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree3(new pcl::search::KdTree<pcl::PointNormal>);
	tree3->setInputCloud(cloud_with_normals);   //点云构建搜索树

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //定义三角化对象
	pcl::PolygonMesh triangles;                //存储最终三角化的网络模型

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);  //设置连接点之间的最大距离，（即是三角形最大边长）

	// 设置各参数值
	gp3.setMu(2.5);  //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
	gp3.setMaximumNearestNeighbors(100);    //设置样本点可搜索的邻域个数
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45
	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10
	gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120
	gp3.setNormalConsistency(false);  //设置该参数保证法线朝向一致

	// Get result
	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree3);   //设置搜索方式
	gp3.reconstruct(triangles);  //重建提取三角化
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_triangles(new pcl::PointCloud<pcl::PointXYZ>);
	fromPCLPointCloud2(triangles.cloud, *cloud_triangles);
	*cloud = *cloud_triangles;
	sensor_msgs::PointCloud2 cloud_greedy;
	pcl::toROSMsg(*cloud_triangles, cloud_greedy);
	cloud_greedy.header.frame_id = "rslidar";
	pub_cloud_greedy.publish(cloud_greedy);


    //欧式聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);;
    tree1->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;				       	//创建点云索引向量，用于存储实际的点云信息
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;	                //欧式聚类对象
    ec.setClusterTolerance (0.30);								    	//设置近邻搜索的搜索半径，这里单位为m
    ec.setMinClusterSize (point_min);								            //设置一个聚类需要的最少的点数目为2000
    ec.setMaxClusterSize (point_max);								      	//设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree1);								        	//设置点云的搜索机制
    ec.setInputCloud (cloud);					                        //输入需要聚类的点云指针地址
    ec.extract (cluster_indices);								      	//得到2000-25000点的聚类

    std::cerr << "聚类数量: " << cluster_indices.size() << "\n" << std::endl;

    //为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中
    //迭代访问点云索引cluster_indices,直到分割出所有聚类
    int cluster_count = 0, cluster_count_small = 0, cluster_count_big=0;
    int j =0,k=0;
	double centroid_big_x, centroid_big_y;
    std::vector<double> centroid_x, centroid_y, centroid_z, use_x, use_y,big_x,big_y;
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

            double sum_x = 0,sum_y = 0,sum_z = 0;
            for(int i = 0; i < cloud_cluster->points.size(); i++)
            {
                sum_x += cloud_cluster->points[i].x;
                sum_y += cloud_cluster->points[i].y;
                sum_z += cloud_cluster->points[i].z;
            }
//z轴限制
            centroid_x.push_back( sum_x / cloud_cluster->points.size() );
            centroid_y.push_back( sum_y / cloud_cluster->points.size() );
            centroid_z.push_back( sum_z / cloud_cluster->points.size() );

        pcl::PointXYZ min;//用于存放三个轴的最小值
        pcl::PointXYZ max;//用于存放三个轴的最大值
        pcl::getMinMax3D(*cloud_cluster,min,max);//得到点云x,y,z三个轴上的最大值和最小值

       if((max.z - min.z < 0.35) && (max.z - min.z > 0.05))//提取小锥桶,过滤其他物体，但会出现第三对锥桶闪的情况
      {
            if(max.x - min.x > 0.3 || max.y -min.y > 0.3 )
            {}
            else if(max.z >0.3)
            {}
            else
            {    
                    cluster_count_small++;
                    std::cerr << "Cluster " << cluster_count_small << " has " << cloud_cluster->points.size () << " data points." << std::endl;
                    std::cerr << "x = " << centroid_x[j] << "; y = " << centroid_y[j] << "z = " << centroid_z[j] <<std::endl;
                    use_x.push_back( centroid_x[j] );
                    use_y.push_back( centroid_y[j] );

                    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloud_cluster_small->points.push_back(cloud->points[*pit]);
                    cloud_cluster_small->width = cloud_cluster_small->points.size();
                    cloud_cluster_small->height = 1;
                    cloud_cluster_small->is_dense = true;
            }            
       }
       j++;
	   //判断大锥桶并计数
	   if (max.z > 0.5)
	   {
		   cluster_count_big++;
		   big_x.push_back(centroid_x[k]);
		   big_y.push_back(centroid_y[k]);
		   k++;
	   }
	}
	//若大锥桶数量为4，则输出停止信号
	if (cluster_count_big == 2)
	{
		centroid_big_x = ((big_x[0] + big_x[1] ) / 2);
		//centroid_big_y = ((big_y[0] + big_y[1] + big_y[2] + big_y[3]) / 4);
		if (centroid_big_x<=1)
		{
			std::cerr << "The stop signal." << std::endl;
		}
	}
    //1000表示没有锥桶
       
        rfans_driver::point_zx point_msg;    
        for (int i = cluster_count_small ; i < 7 ; i++)
        {
         use_x.push_back( 1000 );
         use_y.push_back( 1000 );
        }
        point_msg.x1 = use_x[0];
        point_msg.y1 = use_y[0];
        point_msg.x2 = use_x[1];
        point_msg.y2 = use_y[1];
        point_msg.x3 = use_x[2];
        point_msg.y3 = use_y[2]; 
        point_msg.x4 = use_x[3];
        point_msg.y4 = use_y[3]; 
        point_msg.x5 = use_x[4];
        point_msg.y5 = use_y[4]; 
        point_msg.x6 = use_x[5];
        point_msg.y6 = use_y[5]; 
        
        pub_point.publish(point_msg);

    *cloud=*cloud_cluster_small;
    std::cerr << "小锥桶数量: " << cluster_count_small << "\n" << std::endl;

    sensor_msgs::PointCloud2 cloud_output_small;
    pcl::toROSMsg(*cloud_cluster_small, cloud_output_small);
    cloud_output_small.header.frame_id = "rslidar";
    pub_cloud_output_small.publish(cloud_output_small);
	
  }

private:
  ros::NodeHandle n;
  ros::Publisher pub_cloud_filtered;
  ros::Publisher pub_cloud_re;
  ros::Publisher pub_cloud_other;
  //ros::Publisher pub_cloud_output_projected;
  //ros::Publisher pub_cloud_output_final;
  ros::Publisher pub_cloud_output_small;
  ros::Publisher pub_point;
  ros::Publisher pub_cloud_greedy;
  
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


