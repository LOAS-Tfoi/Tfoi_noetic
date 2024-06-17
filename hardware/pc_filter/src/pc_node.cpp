#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <iostream>


class PCFilters
{
public:
  PCFilters()
  {
    
    pc_subscriber_1_ = n_.subscribe("/camera1/depth/depth2pc", 5, &PCFilters::callback, this);// /camera1/camera/depth/color/points
    pc_publisher_1_ = n_.advertise<sensor_msgs::PointCloud2>("/camera1/depth/color/filterspoints", 5);///camera1/depth/depth2pc/filterspoints

    pc_subscriber_2_ = n_.subscribe("/camera2/depth/depth2pc", 5, &PCFilters::callback_1, this);///camera2/camera/depth/color/points
    pc_publisher_2_ = n_.advertise<sensor_msgs::PointCloud2>("/camera2/depth/color/filterspoints", 5);
    
    ros::NodeHandle nh("~");
    nh.param<double>("rotate_x_1", rotate_x_1_, 0.0);
    nh.param<double>("rotate_y_1", rotate_y_1_, 0.0);
    nh.param<double>("rotate_z_1", rotate_z_1_, 0.0);

    nh.param<double>("rotate_x_2", rotate_x_2_, 0.0);
    nh.param<double>("rotate_y_2", rotate_y_2_, 0.0);
    nh.param<double>("rotate_z_2", rotate_z_2_, 0.0);

    transform_1_ = Eigen::Affine3f::Identity();
    transform_2_ = Eigen::Affine3f::Identity();
    transform_3_ = Eigen::Affine3f::Identity();

    transform_4_ = Eigen::Affine3f::Identity();
    transform_5_ = Eigen::Affine3f::Identity();
    transform_6_ = Eigen::Affine3f::Identity();

    transform_1_.rotate(Eigen::AngleAxisf(rotate_x_1_, Eigen::Vector3f::UnitX()));

    transform_2_.rotate(Eigen::AngleAxisf(rotate_z_1_, Eigen::Vector3f::UnitZ()));

    transform_3_.rotate(Eigen::AngleAxisf(rotate_y_1_, Eigen::Vector3f::UnitY()));

    transform_3_ = transform_2_ * transform_1_; //* transform_3;


    transform_4_.rotate(Eigen::AngleAxisf(rotate_x_2_, Eigen::Vector3f::UnitX()));

    transform_5_.rotate(Eigen::AngleAxisf(rotate_z_2_, Eigen::Vector3f::UnitZ()));

    transform_6_.rotate(Eigen::AngleAxisf(rotate_y_2_, Eigen::Vector3f::UnitY()));

    transform_6_ = transform_5_ * transform_4_; //* transform_6;

    cloud_filtered_1_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    transformed_cloud_1_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in_1_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_filtered_2_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    transformed_cloud_2_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in_2_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  }
  
  void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::fromROSMsg(*msg , *cloud_in_1_);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in_1_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);//距离裁剪
    pass.filter (*cloud_filtered_1_);


    pcl::VoxelGrid<pcl::PointXYZ> sor0;
    sor0.setInputCloud (cloud_filtered_1_);
    sor0.setLeafSize (0.1, 0.1, 0.1);//体素滤波
    sor0.filter (*cloud_filtered_1_);


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_filtered_1_);
    sor.setMeanK (10); //设置考虑查询点临近点数
    sor.setStddevMulThresh (0.5);//设置判断是否为离群点的阀值0.5
    sor.filter (*cloud_filtered_1_);



    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_filtered_1_);
    outrem.setRadiusSearch(0.1);//设置半径为0.8的范围内找临近点0.2
    outrem.setMinNeighborsInRadius (2);//设置查询点的邻域点集数小于5的删除2
    outrem.filter (*cloud_filtered_1_);


    pcl::transformPointCloud(*cloud_filtered_1_, *transformed_cloud_1_, transform_3_);
	

    sensor_msgs::PointCloud2 output_msg;
    output_msg.header.stamp=ros::Time::now();
	
    pcl::toROSMsg(*transformed_cloud_1_, output_msg);
    output_msg.header.frame_id = "camera_filters_1";
    pc_publisher_1_.publish(output_msg);
  }


  void callback_1(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::fromROSMsg(*msg , *cloud_in_2_);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_in_2_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);//距离裁剪
    pass.filter (*cloud_filtered_2_);


    pcl::VoxelGrid<pcl::PointXYZ> sor0;
    sor0.setInputCloud (cloud_filtered_2_);
    sor0.setLeafSize (0.1, 0.1, 0.1);//体素滤波
    sor0.filter (*cloud_filtered_2_);


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_filtered_2_);
    sor.setMeanK (10); //设置考虑查询点临近点数
    sor.setStddevMulThresh (0.5);//设置判断是否为离群点的阀值
    sor.filter (*cloud_filtered_2_);



    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_filtered_2_);
    outrem.setRadiusSearch(0.2);//设置半径为0.8的范围内找临近点
    outrem.setMinNeighborsInRadius (2);//设置查询点的邻域点集数小于5的删除
    outrem.filter (*cloud_filtered_2_);




    pcl::transformPointCloud(*cloud_filtered_2_, *transformed_cloud_2_, transform_6_);
	

    sensor_msgs::PointCloud2 output_msg;
    output_msg.header.stamp=ros::Time::now();
	
    pcl::toROSMsg(*transformed_cloud_2_, output_msg);
    output_msg.header.frame_id = "camera_filters_2";
    pc_publisher_2_.publish(output_msg);
  }
  
private:
  ros::NodeHandle n_;
  ros::Subscriber pc_subscriber_1_,pc_subscriber_2_;
  ros::Publisher pc_publisher_1_,pc_publisher_2_;
  double rotate_x_1_,rotate_y_1_,rotate_z_1_,rotate_x_2_,rotate_y_2_,rotate_z_2_;
  std::string sub_topic_;

  Eigen::Affine3f transform_1_;
  Eigen::Affine3f transform_2_;
  Eigen::Affine3f transform_3_;

  Eigen::Affine3f transform_4_;
  Eigen::Affine3f transform_5_;
  Eigen::Affine3f transform_6_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_1_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_1_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_2_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_2_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pc_filter_node");
  PCFilters pc;

  sleep(3);
  ros::spin();
  return 0;
}



