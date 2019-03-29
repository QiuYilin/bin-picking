#include "ros/ros.h"
#include "cood_tran.h"

#define coordinateDebug 0

int u,v;

float camera_x, camera_y, camera_z;

std::string obj_class,target_obj;


void coodTran(ros::NodeHandle nh)
{
  target_obj = "None";
  ros::param::get("target", target_obj);

  ros::ServiceServer car_interact_server =
  nh.advertiseService("car_interact_srv", carInteract);

  ros::Subscriber ros_coord_pixel_sub =
  nh.subscribe("/darknet_ros/bounding_boxes", 1, darknetCallback);

  ros::Subscriber point_cloud_sub =
  nh.subscribe("camera/depth_registered/points", 1, pointCouldCallback);///camera/depth_registered/points  /camera/depth_registered/points<->color_optical
  
  ros::ServiceServer location_server =
  nh.advertiseService("location_srv", location);
  ROS_INFO("ready location server");
}


bool carInteract(cood_tran_msgs::car_interact::Request &req,
              cood_tran_msgs::car_interact::Response &res) 
{
  
  return true;
}

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
  if (point_pcl.isOrganized ())
  {
    auto pt = point_pcl.at(u,v);
    //旧版本的realsense包乘以0.124987系数
    camera_x = 0.124987*pt.x;
    camera_y = 0.124987*pt.y;
    camera_z = 0.124987*pt.z;
  }
  else
  std::cout << " the pointcloud is not organized " << std::endl;


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

bool location(cood_tran_msgs::location::Request &req,
              cood_tran_msgs::location::Response &res) 
{
  std::cout << "obj_class " << obj_class <<std::endl;
  if (obj_class != target_obj)     
       res.arm_on = 0;    
  else
  {
    if(camera_z >= 0 && camera_z <= 1.5)
    {
      std::cout << "Target get."<< std::endl;
      //乘以转换矩阵得到final_x final_y final_z
      float final_x, final_y, final_z;
      final_x = camera_x;
      final_y = camera_y;
      final_z = camera_z;
      float R2 = final_x*final_x + final_y*final_y;
      //判断初步识别到的位置是否在机械臂工作空间
      if (R2>= 0.2*0.2 && R2<=0.256*0.256)
      {
        res.arm_on = 1;
        res.x = final_x;
        res.y = final_y;
        res.z = final_z;
        std::cout << "x  = " << camera_x << " y = " << camera_y
                  << " z = " << camera_z << std::endl;
      }
      else
      {
        res.arm_on = 0;
        res.dx = -1*final_x;
        res.dy = final_y - 0.256 + 0.01;
      }
    }
    else
    {
     res.arm_on = 0;
     std::cout << "bad parameters " << obj_class <<std::endl;
    }
  }
  return true;
}