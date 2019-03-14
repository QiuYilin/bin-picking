本功能包依托realsense2_camera和darknet_ros

                             # 一、配置realsense2_camera功能包
参见：https://github.com/QiuYilin/ros-realsense2-install

                              # 二、配置darknet_ros功能包
参见： https://github.com/leggedrobotics/darknet_ros
                                 # 三、配置dobot
将dobot文件夹放到工作空间内

1.设置串口权限 sudo usermod -a -G dialout username //其中,username为普通用户名,请根据实际情况替换。 2.修改工作空间目录权限 cd /home/dobot-ws sudo chmod 777 ./* -R

                                          # 启动方法
将这四种功能包一同放到workspace编译,执行
...
roslaunch realsense2_camera rs_rgbd.launch
roslaunch darknet_ros darknet_ros.launch
roslaunch cood_tran coo_tran.launch target:=red (red为可选目标)
roslaunch dobot dobot.launch
...
                                          # 更新
2019.3.13 SR300应该避免在20cm以内的地方测距，其点云单位存在问题，暂且搁置，可能会查看一下重新标定的方法，但标定和深度信息应该没影响  眼在手上采用物理转换关系比较好，原点应该是光学主点，主点和焦距在内参里得到。

2019.3.14 目前保险的方案，安装旧版本的realsense包 2.0.4,本身SR300是老旧产品，后期更新都是针对新产品的。但是效果差不多，所以先乘以0.124987，大概是对的吧我想。 

接下来的工作： 1.关注点云测距问题。 2.周末测试。








