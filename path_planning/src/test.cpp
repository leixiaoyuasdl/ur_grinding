/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);



    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = arm.getCurrentJointValues().at(0)+0.1;
    joint_group_positions[1] = arm.getCurrentJointValues().at(1);
    joint_group_positions[2] = arm.getCurrentJointValues().at(2);
    joint_group_positions[3] = arm.getCurrentJointValues().at(3);
    joint_group_positions[4] = arm.getCurrentJointValues().at(4);
    joint_group_positions[5] = arm.getCurrentJointValues().at(5)+0.1;

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    sleep(1);



    ros::shutdown();

    return 0;
}