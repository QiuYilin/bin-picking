#ifndef COORDINATETRAN_H
#define COORDINATETRAN_H

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <coord_tran_msgs/location.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h> //点云头文件,用于发布接受pcl对象
#include <pcl_conversions/pcl_conversions.h> //点云转换头文件
#include <sensor_msgs/PointCloud2.h>         //消息头文件
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include "ros/ros.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <image_transport/image_transport.h>//image_transport包用于发布订阅图片
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>//这两个包含了CvBridge类
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>//opencv模块


#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>


#include <pcl/filters/statistical_outlier_removal.h>//统计滤波头文件

#include <pcl/filters/filter.h>

//矩阵转换坐标系

#include <Eigen/Dense>
#include <Eigen/SVD>

#define pictureCatch 0
//最小包围盒用
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/registration/transformation_estimation_svd.h>



namespace huskybot_arm
{

  class CoordinateTran
  {

    public:
    //构造函数
    explicit CoordinateTran(ros::NodeHandle nh);
    virtual ~CoordinateTran()=default;

    protected:

    //ros::NodeHandle nh;
    std::string target_obj;
    int u,v;
    int x_min,x_max,y_min,y_max;
    float middle_x,middle_y,middle_z;
    std::vector<float> camera_location;
    int get_target;


    pcl::visualization::CloudViewer viewer;
    ros::Subscriber ros_coord_pixel_sub;

    ros::Subscriber find_obj_sub ;

    ros::Subscriber point_cloud_sub;///camera/depth_registered/points  /camera/depth_registered/points<->color_optical


#if   pictureCatch
    
    image_transport::Subscriber picture_sub;
    
    

    void pictureCallback(const sensor_msgs::Image::ConstPtr &image_msg);
#endif 
    
    ros::ServiceServer location_server;

    ros::Publisher pcl_pub;

    void findObjCallback(const std_msgs::Int8::ConstPtr &msg);

    void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);

    virtual void pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg);

    bool location(coord_tran_msgs::location::Request &req,coord_tran_msgs::location::Response &res);
    
    std::vector<float> threeDdeal (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
  };


  // class CoordinateTran3D :public CoordinateTran
  // {
  //   public:
  //   explicit CoordinateTran3D(ros::NodeHandle nh):CoordinateTran(nh);  

  //   private:

  //   void pointCouldCallback override( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg);
  // //如何定义派生类的虚函数
  // };


}

#endif
