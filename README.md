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


启动方法
```
roslaunch realsense2_camera rs_rgbd.launch
roslaunch darknet_ros darknet_ros.launch
rosrun coo_tran coo_tran
rosrun dobot DobotServer
rosrun dobot DobotClient_PTP
```

目前的问题
1.通过rgbd得到的点云测距不准确
2.手眼标定

接下来的工作：
1.解决点云测距问题。
2.询问手眼标定方法, 可以先写成固定点 测试机械臂控制逻辑是否正确。
3.将dobot两个节点写成一个launch。







