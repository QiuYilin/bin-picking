本功能包依托realsense2_camera和darknet_ros

                            # 一、配置realsense2_camera功能包
参见：https://github.com/QiuYilin/ros-realsense2-install

sudo apt install ros-kinetic-rgbd-launch

                              # 二、配置darknet_ros功能包
参见： https://github.com/leggedrobotics/darknet_ros
                                 # 三、配置dobot
将dobot文件夹放到工作空间内

1.设置串口权限 sudo usermod -a -G dialout username //其中,username为普通用户名,请根据实际情况替换。 2.修改工作空间目录权限 cd /home/dobot-ws sudo chmod 777 ./* -R

                                          # 启动方法
将这四种功能包一同放到workspace编译,执行
```
roslaunch realsense2_camera rs_rgbd.launch
roslaunch darknet_ros darknet_ros.launch
roslaunch cood_tran coo_tran.launch target:=red (red为可选目标)
roslaunch dobot dobot.launch

rosservice call arm_car_srv   (执行一次机械臂抓取任务)
```
                                         #  更新
2019.3.13 SR300应该避免在20cm以内的地方测距，其点云单位存在问题，暂且搁置，可能会查看一下重新标定的方法，但标定和深度信息应该没影响  眼在手上采用物理转换关系比较好，原点应该是光学主点，主点和焦距在内参里得到。

2019.3.14 目前保险的方案，安装旧版本的realsense包 2.0.4,本身SR300是老旧产品，后期更新都是针对新产品的。但是效果差不多，所以先乘以0.124987，大概是对的吧我想。 

2019.3.25 realsense包已经更新，所有坐标单位都是mm :https://github.com/intel-ros/realsense/pull/683,即需要更新到新的包且不要乘以0.124987的系数。需要把这个版本的包保存下来。


2019.3.27 确立代码风格   位姿估计对四自由度机器人的意义不大 目前最需要的是增加实际场景交互逻辑
http://community.bwbot.org/topic/468/ros-c-%E4%BB%A3%E7%A0%81%E9%A3%8E%E6%A0%BC%E8%AF%B4%E6%98%8E
https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/#id2

2019.4.10 Huskybo_arm/cood_tran/src/Coordinate.cpp 120 坐标转换 










