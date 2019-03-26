#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <location_srv/Location.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h> //点云头文件,用于发布接受pcl对象
#include <pcl_conversions/pcl_conversions.h> //点云转换头文件
#include <sensor_msgs/PointCloud2.h>         //消息头文件
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>



void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
void pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg);
bool location(location_srv::Location::Request &req,location_srv::Location::Response &res);