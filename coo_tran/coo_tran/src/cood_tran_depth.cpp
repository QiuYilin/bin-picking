/* 
# The coodinate can be caculated by depth extrinsic and intrinsic
#include <ros/ros.h>
#include <image_transport/image_transport.h>//image_transport包用于发布订阅图片
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>//这两个包含了CvBridge类
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>//opencv模块
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
float u,v;
std::string obj_class;
void darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)

{
  
  //ROS_INFO("darknetCallback");
  obj_class = msg->bounding_boxes[0].Class;
  
  u = (msg->bounding_boxes[0].xmin + msg->bounding_boxes[0].xmax) / 2;
  v = (msg->bounding_boxes[0].ymin + msg->bounding_boxes[0].ymax) / 2;

  std::cout << "u0 " << u << std::endl;
  std::cout << "v0 " << v << std::endl;

  // boundingBoxesResults_.bounding_boxes.clear();
  // std::cout << "\033[2J\033[1;1H";     // clear terminal
}

void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
  cv_bridge::CvImagePtr depth_ptr;

    
  depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1); 
//   int pic_type = depth_ptr->image.type();
//   std::cout << "the element type of depth_pic is " << pic_type << std::endl;
  auto distance = depth_ptr->image.at<ushort>(v,u);
  std::cout << "distance: "  << distance <<std::endl;     // clear terminal
  u = 0;
  v = 0;

}

int main(int argc, char **argv) {
  //初始化ROS
  ros::init(argc, argv, "cood_tran_depth");
  ros::NodeHandle nh;
    //创建接收darknet结果和点云信息的接受者
  ros::Subscriber ros_coord_pixel_sub =
      nh.subscribe("/darknet_ros/bounding_boxes", 1, darknetCallback);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub1 = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);

  ros::spin();
  return 0;
}
*/