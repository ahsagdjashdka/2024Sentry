
### 2024赛季哨兵导航部分代码> 

#### 介绍
使用[深北莫北极熊开源](https://gitee.com/SMBU-POLARBEAR/pb_rm_simulation),本文档中的代码未随原开源更新，不是最新版本。
#### 代码框架
```
└──rm_navigation_ws                                 
   └── src
       ├──rm_bringup
          ├──config
          |  ├──reality
          |  └──simulation
          ├──launch
          |  ├──bringup_real.launch.py
          |  └──bringup_sim.launhc.py
          ├──map
          ├──pcd
          ├──rviz
          ├──urdf
          ├──Cmakelists.txt
          └──package.xml
        ├──rm_driver\livox_ros_driver2
           └──src
        ├──rm_localization
           ├──FAST_LIO
           ├──icp_registration
           └──point_lio
       ├──rm_navigation
          ├──costmap_converter
          ├──fake_vel_transform
          ├──rm_navigation
          └──teb_local_planner
       ├──rm_perception
          ├──imu_complementtary_filter
          ├──linefit_ground_segementation_ros2
          ├──pointcloud_downsampling
          └──point_cloud_to_laserscan
       └──rm_simulation
          ├──four_wheel_steering_controller
          ├──livox_laser_simulation_ROS2
          ├──pb_rm_simulation
          ├──ranger_mini_v2
          └──ranger_mini_v2_gazebo
```
#### 说明
<<<<<<< HEAD
1. rm_bringup <br>
- 参数配置文件config和运行文件launch（有些参数在此处改动为无效，需要回到原位置修改）
- map中为栅格地图（.data/.pgm/.posegraph/.yaml）  <br>
若使用slam_toolbox，需要提供.posegraph[pcd转pgm/pgm转posegraph](https://flowus.cn/lihanchen/share/00ffde73-9a62-4fdf-ac52-6785f7666b9a)  <br>
若使用icp，需提供点云文件(.pcd)
- pcd为点云文件
- rviz可视化
- urdf模型文件  <br>

2. rm_driver\livox_ros_driver2
- 览沃激光雷达驱动[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)    注：北极熊开源中使用的驱动有修改
3. rm_localization（该模块很重要）
- 包括三种不同里程计算法，icp在当前pc中开销较大，不推荐使用，在最新开源中已被弃用/fast-lio2的实际效果并不足够好，若仍然使用需优化/pointlio未实际测试过。
- 定位算法包括slam_toolbox location模式(通过扫描匹配并优化确定位置，slam_toolbox的不同模式有区别,注意区分)/amcl(使用粒子过滤器追踪姿态，实际测试中拟合速度较慢)/icp(为点云匹配算法，可用于获取初始位姿，无回环检测，容易出现积累误差)
4. rm_navigation 导航模块
- costmap_converter为navigation2中的插件，使用该插件可以将障碍物转换为代价地图
- fake_vel_transform用于解决雷达跟随云台旋转时的速度变换问题
- rm_navigation请参考[navigation2官方文档中的参数](https://docs.nav2.org/plugins/index.html)- teb_local_planner为局部路径规划器
5. rm_perception 感知模块
- [imu_complementtary_filter为imu滤波器，详情见官方文档](https://wiki.ros.org/imu_complementary_filter)
- linefit_ground_segementation_ros2为点云分割算法
- pointcloud_downsampling点云下采样算法(用于解决特征提取时计算成本高的问题)     
- point_cloud_to_laserscan将点云格式由PointCloud2转换为Laserscan
6. rm_simulation为仿真模块
- livox_laser_simulation_ROS2点云仿真
- pb_rm_simulation:RMUC/RMUL中导航仿真
7. save_grid_map.sh可用于保存地图
8. save_pcd.sh可用于保存点云文件
#### contact
qq：2574944170  赵欣