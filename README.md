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

                                   三、配置dobot
将dobot文件夹放到工作空间内

1.设置串口权限
sudo usermod -a -G dialout username //其中,username为普通用户名,请根据实际情况替换。
2.修改工作空间目录权限
 cd /home/dobot-ws
 sudo chmod 777 ./* -R


将这四种功能包一同放到workspace编译,执行

```
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
roslaunch darknet_ros darknet_ros.launch
rosrun coo_tran coo_tran
```






