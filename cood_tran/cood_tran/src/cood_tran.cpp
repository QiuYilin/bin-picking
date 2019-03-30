#include "ros/ros.h"
#include "cood_tran.h"



int u,v;

float camera_x, camera_y, camera_z;

std::string obj_class,target_obj;

void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
  obj_class = "none";
  std::cout << "Items get "  << std::endl;
  obj_class = msg->bounding_boxes[0].Class;
  u = (msg->bounding_boxes[0].xmin + msg->bounding_boxes[0].xmax) / 2;
  v = (msg->bounding_boxes[0].ymin + msg->bounding_boxes[0].ymax) / 2;
  std::cout << "Items refrash "  << std::endl;

  // std::cout << "\033[2J\033[1;1H";     // clear terminal
}

void pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg) 
{
  //std::cout << "pointCloudCallback" << std::endl;
// #if pointCouldDebug
//   std::cout << "pointCloud2 (header):" << point_cloud_msg->header << std::endl;
// #endif

    pcl::PointCloud<pcl::PointXYZ> point_pcl;
    pcl::fromROSMsg(*point_cloud_msg, point_pcl);
    if (point_pcl.isOrganized ())
    {
      if (obj_class == target_obj)
      {
        auto pt = point_pcl.at(u,v);
        //旧版本的realsense包乘以0.124987系数
        camera_x = 0.124987*pt.x;
        camera_y = 0.124987*pt.y;
        camera_z = 0.124987*pt.z;
        std::cout << " coordnate get " << std::endl;
      }
    }
    else
    std::cout << " the pointcloud is not organized " << std::endl;
    //std::cout << "\033[2J\033[1;1H";     // clear terminal


}

bool location(cood_tran_msgs::location::Request &req,
              cood_tran_msgs::location::Response &res) 
{
    std::cout << "          teminal_obj_class " << obj_class <<std::endl;
    if (obj_class != target_obj)     
        res.arm_on = 0;    
    else
    {
        if(camera_z >= 0 && camera_z <= 1.5)
        {
            std::cout << "          terminal target get."<< std::endl;
            //乘以转换矩阵得到final_x final_y final_z
            float final_x, final_y, final_z;
            final_x = camera_x;
            final_y = camera_y;
            final_z = camera_z;
            float R2 = final_x*final_x + final_y*final_y;

            res.x = final_x;
            res.y = final_y;
            res.z = final_z;
            //判断初步识别到的位置是否在机械臂工作空间
            if (R2>= 0.2*0.2 && R2<=0.256*0.256)
            {
              res.arm_on = 1;
              std::cout << "            you can grasp "  <<std::endl;
              
            }
            else
            {
              std::cout << "            Out of workspace"<< std::endl;
              res.arm_on = 0;
              // res.dx = -1*final_x;
              // res.dy = final_y - 0.256 + 0.01;
            }
        }
        else
        {
            res.arm_on = 0;
            std::cout << "bad parameters "  <<std::endl;
        }
    }
    return true;
}