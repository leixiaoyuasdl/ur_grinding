
# 介绍
  智能打磨机器人，相机识别物体表面，并进行打磨轨迹规划。
# 环境
ubuntu 16.04

ros kinetic
# 依赖
```
bezier
https://github.com/ros-industrial-consortium/bezier

UR
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
```
# 使用
```
roslaunch ur_grinding_rviz_plugin ur_grinding.launch
```
![1](https://user-images.githubusercontent.com/13638834/168721581-3a4e8b20-50e2-4696-8eb8-7e330540eeb0.png)

# 其他
![11](https://user-images.githubusercontent.com/13638834/193035602-705d30a7-5be5-4219-a897-05e474d21c62.png)
```
注意：点云和CAD模型都是相对于"base_link"坐标系
```
![21](https://user-images.githubusercontent.com/13638834/193035625-30881297-e913-4c3f-9531-0fb4afdb6163.png)
![4](https://user-images.githubusercontent.com/13638834/193035655-d61a0828-b66d-4459-aa08-4d046a80652e.png)
