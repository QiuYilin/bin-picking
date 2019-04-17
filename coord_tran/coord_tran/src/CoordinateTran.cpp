#include "ros/ros.h"
#include "CoordinateTran.h"




//pcl::visualization::CloudViewer viewer("Cloud in box");


namespace huskybot_arm
{ 

    CoordinateTran::CoordinateTran(ros::NodeHandle nh)
    :viewer("cloud in box"),nh(nh),get_target(0)
    {
    ros::param::get("target", target_obj);

    std::cout << "target_obj_class" << target_obj <<std::endl;

    ROS_INFO("[CoordinateTran] Node started.");


    find_obj_sub =
    nh.subscribe("/darknet_ros/found_object", 1, &CoordinateTran::findObjCallback,this);


    ros_coord_pixel_sub =
    nh.subscribe("/darknet_ros/bounding_boxes", 1, &CoordinateTran::darknetCallback,this);

#if   pictureCatch
    image_transport::ImageTransport it(nh);
    picture_sub = it.subscribe("/camera/color/image_raw", 1, &CoordinateTran::pictureCallback,this);
#endif 

    point_cloud_sub =
    nh.subscribe("/camera/depth_registered/points", 1, &CoordinateTran::pointCouldCallback,this);///camera/depth_registered/points  /camera/depth_registered/points<->color_optical
    
    location_server =
    nh.advertiseService("location_srv", &CoordinateTran::location,this);

    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 100);

    }


    //订阅识别物体数量 get_target 清零
    void CoordinateTran::findObjCallback(const std_msgs::Int8::ConstPtr &msg)
    {
    if (msg->data == 0)
    {
        std::cout << " Find nothing ! " << std::endl;
        std::cout << "\033[2J\033[1;1H";     // clear terminal
        get_target = 0;
    }
    }





    //查找目标物体 得到搜索框坐标信息
    void CoordinateTran::darknetCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
    {
    std::cout << "Find something ! " ;
    
        for (size_t i = 0;i < sizeof(msg->bounding_boxes);i++)
        {
            if (msg->bounding_boxes[i].Class == target_obj)
            {
                std::cout << " and it is  target ! "  << std::endl;
                x_min = msg->bounding_boxes[i].xmin;
                x_max = msg->bounding_boxes[i].xmax;
                y_min = msg->bounding_boxes[i].ymin;
                y_max = msg->bounding_boxes[i].ymax;
                u = (x_min + x_max) / 2;
                v = (y_min + y_max) / 2;
                std::cout << " dark_u "  << u << " dark_v "  << v << std::endl;
                get_target =1;
                break;
            }
            else
            {
                std::cout << " but it is not target ! "  << std::endl;
                get_target =0;
            }
        }
    // std::cout << "\033[2J\033[1;1H";     // clear terminal
    }



#if   pictureCatch
    void CoordinateTran::pictureCallback(const sensor_msgs::Image::ConstPtr &image_msg)
    {

 
        if (get_target == 1)
        {

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8); 



               cv::Mat catchPicture(y_max-y_min +1,x_max-x_min+1,CV_8UC3);
                

                for (int i=0; i< catchPicture.cols&&i+x_min<640; ++i)
                {
                   for (int j=0; j< catchPicture.rows&&j+y_min<480; ++j)
                   {
                       
                    //    std::cout << " i  " << i <<" j "<< j  <<std::endl;
                    //    std::cout << " x_min+i  " << x_min+i <<" y_min+j "<< y_min+j  <<std::endl;
                       catchPicture.at<cv::Vec3b>(j,i) =cv_ptr->image.at<cv::Vec3b>(y_min+j,x_min+i);
                       catchPicture.at<cv::Vec3b>(j,i) =cv_ptr->image.at<cv::Vec3b>(y_min+j,x_min+i);
                       catchPicture.at<cv::Vec3b>(j,i) =cv_ptr->image.at<cv::Vec3b>(y_min+j,x_min+i);

                       //std::cout << "x " << cloud_inbox.points[i].x << " y " << cloud_inbox.points[i].y << " z " << cloud_inbox.points[i].z <<std::endl;
                       
                   }
                }
                
                imwrite("2d.jpg", catchPicture);
                cv::imshow ("view",catchPicture);
                cv::waitKey(10);

       }
    }
#endif





    //将图像坐标转换为相对于相机的坐标
    void CoordinateTran::pointCouldCallback( const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg) 
    {
    // #if pointCouldDebug
    //   std::cout << "pointCloud2 (header):" << point_cloud_msg->header << std::endl;
    // #endif
 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inbox (new pcl::PointCloud<pcl::PointXYZ>);
        cloud_inbox->clear();
        sensor_msgs::PointCloud2 cloud_inbox_msgs;
        if (get_target == 1)
        {
            pcl::PointCloud<pcl::PointXYZ> point_pcl;
            pcl::fromROSMsg(*point_cloud_msg, point_pcl);
            //pcl::PointCloud<pcl::PointXYZ> cloud_inbox;
            
            if (point_pcl.isOrganized ())
            {


                
                cloud_inbox->width =x_max-x_min+1;
                cloud_inbox->height =y_max - y_min +1;
                std::cout << "cloud_inbox.width "<<cloud_inbox->width << "cloud_inbox.height  " << cloud_inbox->height <<std::endl;
                cloud_inbox->points.resize (cloud_inbox->width * cloud_inbox->height);
                std::cout << "new cloud" <<std::endl;
                for (int i=0; i< cloud_inbox->width&&i+x_min<640; ++i)
                {
                   for (int j=0; j< cloud_inbox->height&&j+y_min<480; ++j)
                   {
                       
                    //    std::cout << " i  " << i <<" j "<< j  <<std::endl;
                    //    std::cout << " x_min+i  " << x_min+i <<" y_min+j "<< y_min+j  <<std::endl;
                       cloud_inbox->at(i,j).x =point_pcl.at(x_min+i,y_min+j).x;
                       cloud_inbox->at(i,j).y =point_pcl.at(x_min+i,y_min+j).y;
                       cloud_inbox->at(i,j).z =point_pcl.at(x_min+i,y_min+j).z;

                       //std::cout << "x " << cloud_inbox.points[i].x << " y " << cloud_inbox.points[i].y << " z " << cloud_inbox.points[i].z <<std::endl;
                       
                   }
                }
                pcl::PointXYZ pt = point_pcl.at(u,v);
                //旧版本的realsense包乘以0.124987系数 单位m
                camera_x = pt.x;
                camera_y = pt.y;
                camera_z = pt.z;
                std::cout << " coordnate get: " << " camera_x " <<camera_x <<" camera_y " <<camera_y <<" camera_z " <<camera_z <<std::endl;
                
            
            }
            else
            std::cout << " the pointcloud is not organized " << std::endl;
            //std::cout << "\033[2J\033[1;1H";     // clear terminal
        }
        viewer.showCloud(cloud_inbox);
        pcl::toROSMsg(*cloud_inbox, cloud_inbox_msgs);
        cloud_inbox_msgs.header.frame_id = "camera_color_optical_frame";
        pcl_pub.publish(cloud_inbox_msgs);

    }

    //将相对于相机的坐标转换为相对于机械臂的坐标
    bool CoordinateTran::location(coord_tran_msgs::location::Request &req,
                coord_tran_msgs::location::Response &res) 
    {
        res.get_target = get_target;
        if(get_target ==1)
        { 
            if(!std::isnan(camera_z))
            {
                //乘以转换矩阵得到物体相对于机械臂的坐标 单位米  

                
                
                
                
                float final_x, final_y, final_z;
                

                //最终坐标=模型转换坐标+牌照位置+修正值

                final_x = -camera_y+0.0705+0.20+0.01;
                final_y = -camera_x-0.0027+0.00+0.01;
                final_z = -camera_z+0.1035+0.07-0.005;
                
                res.x = final_x;
                res.y = final_y;
                res.z = final_z;
            }
            else
            {
                std::cout << "bad value "  <<std::endl;
            }
        }
        return true;
    }

}