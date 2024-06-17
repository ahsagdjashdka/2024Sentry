
### 2024赛季哨兵导航部分代码

#### 介绍
使用[深北莫北极熊开源](https://gitee.com/SMBU-POLARBEAR/pb_rm_simulation)
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

