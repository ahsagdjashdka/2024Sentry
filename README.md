### 2024赛季辽宁科技大学COD战队哨兵上位机方案
#### 简介
本赛季哨兵项目主要分为三个模块，由于测试时间较少且前期考虑不充分，各个模块交互极少。本文档主要为记录24赛季哨兵上位机代码，代码中除部分参数文件及决策部分有改动以外，其他基本与开源无太大区别。
##### 环境
软件
- ubuntu22.04
- ROS2-humble
硬件
Livox-mid360激光雷达
12代-i7 1270p
海康相机 MV-CS016-10UC	8mm镜头
#### 代码框架
```
├──rm_decision_ws
|  ├──BrhaviorTree.ROS2
|  ├──rm_behavior_tree
|  └──rm_decision_interfaces
├──rm_navigation_ws
|  └── src
├──rm_vision_ws
   └── src
```
- rm_decision_ws 哨兵决策部分，使用[behavior_tree](https://www.behaviortree.dev/docs/Intro)，可安装[Groot2](https://www.behaviortree.dev/groot/)进行可视化
- rm_navigation_ws 哨兵导航部分，参考 **[深北莫北极熊开源](https://gitee.com/SMBU-POLARBEAR/pb_rm_simulation)**
- rm_decision_ws 哨兵自瞄部分，详见 **[陈君开源rm_vision](https://gitee.com/ustl-cod/cod_vision)** 
#### 使用
##### 安装依赖
```
sudo apt install -y ros-humble-gazebo-*
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-rviz2
sudo apt install -y ros-humble-nav2*
sudo apt install -y ros-humble-pcl-ros
sudo apt install -y ros-humble-pcl-conversions
sudo apt install -y ros-humble-libpointmatcher
sudo apt install -y ros-humble-tf2-geometry-msgs
sudo apt install -y libboost-all-dev
sudo apt install -y libgoogle-glog-dev
sudo apt install -y ros-humble-libg2o
sudo apt install ros-humble-behaviortree-cpp
sudo apt install ros-humble-serial-driver
```
安装[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
##### 克隆
克隆仓库
```
git clone https://gitee.com/ustl-cod/cod_sentry.git
```
编译

```
colcon build --symlink-install
```
运行

```
# 运行自瞄
./vision.sh
# 导航运行
./nav.sh
# 运行决策
./decision.sh
```
#### 记录
在不到一年之前，我得知我在24赛季要开始做哨兵的算法的时候既激动又惶恐，激动是因为我竟然真的就要开始成为一个真正的Rmer开始参与比赛了，惶恐则是因为我当时完全可以说是一个傻子，我刚学会c++、不了解linux、不懂ros、甚至不知道以当时的规则来说哨兵的上位机都需要什么，并且由于我的学长少且忙（当时学长需要准备复活赛），因此入门大部分需要靠自学，那段时光可以说是既痛苦又充实，我只能不停的找开源，看开源，里面看不懂的就去搜（几乎是看两行搜一下），就像是盲人摸石头过河一样，我不确定我能否学会、学会又能否用好，遇到问题需要浪费大量时间在一些无意义的事情上，摸索不到重点，茫然且无助。在我刚入门不久之后（开始做缝合怪），某一天机缘巧合之下发现了北极熊的开源，大致看了之后发现非常适合使用，于是直接“拿来主义”，但是想要有好的效果仅仅拿来主义是完全不够的，我当时还没能完全理解烧饼的完整逻辑，因此每天就是看代码、调参数、学文档，后来由于机械跑路，迟迟调不上实车，我几乎完全失去了信心，觉得我一直做的没意义，没必要（我一直以为正常步骤是写代码->实车调试->测问题->改bug,没想到压根轮不到我），由于进度的原因，永远是在比赛前两天还有赛场上速通视觉（快乐通宵~），这个时候感谢涛哥一直鼓励我（push我，不然我可能直接开摆了），直到进到分区赛赛场的时候，我才真正意识到一直在搞的东西在赛场上发挥出来是一种什么样的感觉，理解了为什么很多人对此趋之若鹜。于是，我又开启了我的25赛季新篇章。
#### 致谢
感谢君佬的rm_vision开源、感谢深北莫陈力瀚同学的开源及帮助、感谢杨涛学长、王柏程学长、朱云飞学长的帮助和鼓励还有COD战队的队友为哨兵做出的贡献

contact：
qq：2574944170 赵欣

