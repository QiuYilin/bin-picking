本功能包依托realsense2_camera和darknet_ros
                                     


                                 #一、配置realsense2_camera功能包

参见：https://github.com/QiuYilin/ros-realsense2-install

					
                                     #三、配置dobot
将dobot文件夹放到工作空间内

1.设置串口权限
sudo usermod -a -G dialout username //其中,username为普通用户名,请根据实际情况替换。
2.修改工作空间目录权限
 cd /home/dobot-ws
 sudo chmod 777 ./* -R

                                              #启动方法
将这四种功能包一同放到workspace编译,执行                                         
```
roslaunch realsense2_camera rs_rgbd.launch
roslaunch darknet_ros darknet_ros.launch
rosrun coo_tran coo_tran
roslaunch dobot dobot.launch
```
                                              #更新
2019.3.13
SR300应该避免在20cm以内的地方测距，其点云单位存在问题，暂且搁置，可能会查看一下重新标定的方法，但标定和深度信息应该没影响。



接下来的工作：
1.关注点云测距问题。
2.询问手眼标定方法, 可以先写成固定点 测试机械臂控制逻辑是否正确，周末测试。








