#include "ros/ros.h"
#include "cood_tran.h"

#define pointCouldDebug 0
#define coordinateDebug 0

int u,v;

float camera_x, camera_y, camera_z;

std::string obj_class,target_obj;

void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
  obj_class = msg->bounding_boxes[0].Class;
  u = (msg->bounding_boxes[0].xmin + msg->bounding_boxes[0].xmax) / 2;
  v = (msg->bounding_boxes[0].ymin + msg->bounding_boxes[0].ymax) / 2;
  #if coordinateDebug
  std::cout << "u0 " << u << std::endl;
  std::cout << "v0 " << v << std::endl;
  #endif
  // std::cout << "\033[2J\033[1;1H";     // clear terminal
}

void pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg) 
{
#if pointCouldDebug
  std::cout << "pointCloud2 (header):" << point_cloud_msg->header << std::endl;
#endif
#if coordinateDebug
  std::cout << "u " << u << std::endl;
  std::cout << "v " << v << std::endl;
#endif
  pcl::PointCloud<pcl::PointXYZ> point_pcl;
  pcl::fromROSMsg(*point_cloud_msg, point_pcl);
#if pointCouldDebug
  std::cout << "cloud: width = " << point_pcl.width
            << " height = " << point_pcl.height << std::endl;
#endif
  auto pt = point_pcl.at(u,v);
  camera_x = 0.124987*pt.x;
  camera_y = 0.124987*pt.y;
  camera_z = 0.124987*pt.z;
  // std::cout << "c_x  = " << 0.124987*point_pcl.at(0,240).x << " c_y = " << 0.124987*point_pcl.at(0,240).y
  //         << " c_z = " << 0.124987*point_pcl.at(0,240).z << std::endl;
   if (obj_class == target_obj &&(camera_x >= -1.5 ||camera_x<=1.5))
  {
  std::cout << "x  = " << camera_x << " y = " << camera_y
            << " z = " << camera_z << std::endl;
  //std::cout << "\033[2J\033[1;1H";     // clear terminal
  }
  obj_class = "none";
}

bool location(location_srv::Location::Request &req,
              location_srv::Location::Response &res) 
{
  std::cout << "obj_class " << obj_class <<std::endl;
  if (obj_class != target_obj)     
       res.arm_on = 0;    
    else
    if(camera_x >= -1.5 && camera_x <= 1.5)
    {
      std::cout << "Target get."<< std::endl;
      res.arm_on = 1;
      //乘以转换矩阵得到final_x final_y final_z
      float final_x, final_y, final_z;
      final_x = camera_x;
      final_y = camera_y;
      final_z = camera_z;
      res.x = final_x;
      res.y = final_y;
      res.z = final_z;
      std::cout << "x  = " << camera_x << " y = " << camera_y
                << " z = " << camera_z << std::endl;
      std::cout << "service answer" << std::endl;
    }
  return true;
}