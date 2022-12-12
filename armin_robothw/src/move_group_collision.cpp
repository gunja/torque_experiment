/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();


  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success ;

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.4;
  primitive.dimensions[1] = 1.4;
  primitive.dimensions[2] = 0.234;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.02;
  box_pose.position.y = 0.75 + primitive.dimensions[1]/2;
  box_pose.position.z = primitive.dimensions[2] /2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  moveit_msgs::CollisionObject collision_object2;
  collision_object2.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object2.id = "box2";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive2.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 0.8;
  primitive2.dimensions[1] = 0.3;
  primitive2.dimensions[2] = 0.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box2_pose;
  box2_pose.orientation.w = 1.0;
  box2_pose.position.x = -0.1;
  box2_pose.position.y = 1.1;
  box2_pose.position.z = .4;

  collision_object2.primitives.push_back(primitive2);
  collision_object2.primitive_poses.push_back(box2_pose);
  collision_object2.operation = collision_object2.ADD;

  moveit_msgs::CollisionObject collision_object3;
  collision_object3.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object3.id = "wall";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive3;
  primitive3.type = primitive3.BOX;
  primitive3.dimensions.resize(3);
  primitive3.dimensions[0] = 0.1;
  primitive3.dimensions[1] = 5.1;
  primitive3.dimensions[2] = 4.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box3_pose;
  box3_pose.orientation.w = 1.0;
  box3_pose.position.x = -1.3;
  box3_pose.position.y = 0.0;
  box3_pose.position.z = .0;

  collision_object3.primitives.push_back(primitive3);
  collision_object3.primitive_poses.push_back(box3_pose);
  collision_object3.operation = collision_object3.ADD;

  moveit_msgs::CollisionObject collision_object4;
  collision_object4.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object4.id = "floor";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive4;
  primitive4.type = primitive4.BOX;
  primitive4.dimensions.resize(3);
  primitive4.dimensions[0] = 5.3;
  primitive4.dimensions[1] = 6.1;
  primitive4.dimensions[2] = 0.2;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box4_pose;
  box4_pose.orientation.w = 1.0;
  box4_pose.position.x = 0.0;
  box4_pose.position.y = 0.0;
  box4_pose.position.z = -.1;

  collision_object4.primitives.push_back(primitive4);
  collision_object4.primitive_poses.push_back(box4_pose);
  collision_object4.operation = collision_object4.ADD;


  moveit_msgs::CollisionObject collision_object5;
  collision_object5.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object5.id = "table1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive5;
  primitive5.type = primitive5.BOX;
  primitive5.dimensions.resize(3);
  primitive5.dimensions[0] = 0.8;
  primitive5.dimensions[1] = 1.1;
  primitive5.dimensions[2] = 0.92;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box5_pose;
  box5_pose.orientation.w = 1.0;
  box5_pose.position.x = 0.8 + primitive5.dimensions[0]/2.;
  box5_pose.position.y = -0.73 - primitive5.dimensions[1]/2.;
  box5_pose.position.z = primitive5.dimensions[2]  /2 ;

  collision_object5.primitives.push_back(primitive5);
  collision_object5.primitive_poses.push_back(box5_pose);
  collision_object5.operation = collision_object5.ADD;

  moveit_msgs::CollisionObject collision_object6;
  collision_object6.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object6.id = "pump";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive6;
  primitive6.type = primitive6.BOX;
  primitive6.dimensions.resize(3);
  primitive6.dimensions[0] = 0.4;
  primitive6.dimensions[1] = 0.4;
  primitive6.dimensions[2] = 0.57;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box6_pose;
  box6_pose.orientation.w = 1.0;
  box6_pose.position.x = 1. + primitive6.dimensions[0]/2.;
  box6_pose.position.y =  - primitive6.dimensions[1]/2.;
  box6_pose.position.z = primitive6.dimensions[2]  /2 ;

  collision_object6.primitives.push_back(primitive6);
  collision_object6.primitive_poses.push_back(box6_pose);
  collision_object6.operation = collision_object6.ADD;


  moveit_msgs::CollisionObject collision_object7;
  collision_object7.header.frame_id = move_group.getPlanningFrame();
  // The id of the object is used to identify it.
  collision_object7.id = "table1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive7;
  primitive7.type = primitive5.BOX;
  primitive7.dimensions.resize(3);
  primitive7.dimensions[0] = 0.8;
  primitive7.dimensions[1] = 1.1;
  primitive7.dimensions[2] = 2.92;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box7_pose;
  box7_pose.orientation.w = 1.0;
  box7_pose.position.x = 0.8 + primitive5.dimensions[0]/2.;
  box7_pose.position.y =  - primitive5.dimensions[1]/2.;
  box7_pose.position.z = primitive5.dimensions[2]  /2 ;

  collision_object7.primitives.push_back(primitive7);
  collision_object7.primitive_poses.push_back(box7_pose);
  collision_object7.operation = collision_object7.ADD;


  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  collision_objects.push_back(collision_object2);
  collision_objects.push_back(collision_object3);
  collision_objects.push_back(collision_object4);
  collision_objects.push_back(collision_object5);
  collision_objects.push_back(collision_object6);
  collision_objects.push_back(collision_object7);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);


 return 0;
}
