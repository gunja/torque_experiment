#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <signal.h>
#include <errno.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "armin_cyclic_move_node");
    moveit::planning_interface::MoveGroupInterface move_group_int("manipulator");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ros::Duration period(1.0/200); // 200Hz update rate

    ros::Publisher chatter_pub = nh.advertise<trajectory_msgs::JointTrajectory>
        ("/armin/controller/position/command", 1);


    std::list<std::vector<double> > tasks {
    { {0.354, 0.842, 0.439, 0.558, 0.448, 0.453, 0.532}},
    { {-0.019, 1.147, 0.618,  -0.690, -0.022, 0.717, -0.096}},
    { {-0.038, 0.864, 0.488, -0.693, -0.044, 0.708, -0.128}}
    };

    for(auto i=0; i < 5; ++i)
    {
        for( auto const &t : tasks) {

            std::cout<<"Iteration "<<i<<"  of 5 and target "<<t[0]<<" "<<t[1]<<std::endl;
            move_group_int.setStartStateToCurrentState();
            geometry_msgs::Pose target_pose3 = move_group_int.getCurrentPose().pose;
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(target_pose3);
            geometry_msgs::Pose tt;
            tt.position.x = t[0];
            tt.position.y = t[1];
            tt.position.z = t[2];
            tt.orientation.x = t[3];
            tt.orientation.y = t[4];
            tt.orientation.z = t[5];
            tt.orientation.w = t[6];
            waypoints.push_back(tt);

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group_int.computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory);
        std::cout<<"Moving from "<<waypoints[0]<< " to "<<waypoints[1]<<"\n"
            <<"  Prodused a result : "<<fraction<<" and trajectory "<<
                trajectory<<std::endl;
 
            if ( fraction > 0.3)
                chatter_pub.publish(trajectory.joint_trajectory);

            std::cout<<"press any key"<<std::endl;
            std::string tmp;
            std::cin>>tmp;
        }
    }

    spinner.stop();
return 0;
}


