本功能包依托realsense2_camera和darknet_ros
                                     


                                 一、配置realsense2_camera功能包

参见：https://github.com/QiuYilin/ros-realsense2-install

					
                                     二、配置darknet_ros
参见：https://github.com/QiuYilin/darknet_ros_install/tree/master
修改darknet_ros的ros.yaml文件 
subscribers:

  camera_reading:
    topic: /camera/color/image_raw
    queue_size: 1



将这三种功能包一同放到workspace编译,执行

```
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
roslaunch darknet_ros darknet_ros.launch
rosrun coo_tran coo_tran
```






