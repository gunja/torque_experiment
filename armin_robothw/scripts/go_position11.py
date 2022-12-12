# python2 ?
import sys
import copy

import roslib
import rospy
import actionlib
import moveit_commander

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import PlanningScene, ObjectColor

def main():
    JOINT_NAMES = ['a1_joint', 'a2_joint', 'a3_joint', 'a4_joint', 'a5_joint', 'a6_joint']

    rospy.init_node( 'cyclic_several_moves', anonymous=False, disable_signals=True)
    client = actionlib.SimpleActionClient( '/armin/controller/position/follow_joint_trajectory',
                    FollowJointTrajectoryAction)
    client.wait_for_server()
    scene_pub = rospy.Publisher( 'planning_scene', PlanningScene, queue_size=1)
    moveit_commander.roscpp_initialize( sys.argv)
    group = moveit_commander.MoveGroupCommander( "base")
    group.allow_replanning( True)
    group.set_planning_time( 10.0)
    group.set_goal_joint_tolerance( 0.001)
    group.set_end_effector_link('ee_link')

    group.set_goal_position_tolerance( 0.01)
    group.set_goal_orientation_tolerance( 0.01)

    pose = group.get_current_pose().pose
    '''
    end POSE =  position: 
  x: -0.297664274529
  y: 0.86231809977
  z: 0.479473918708
orientation: 
  x: -0.999925816818
  y: 2.57453764782e-05
  z: -0.00223501897456
  w: 0.0119735077817




    '''



    pose.position.x = -0.297663892612 
    pose.position.y = 0.86231809977
    pose.position.z = 0.479473918708 

    pose.orientation.x = -0.999925816818
    pose.orientation.y = 2.57453764782e-05
    pose.orientation.z = -0.00223501897456
    pose.orientation.w = 0.0119735077817

    group.set_pose_target(pose);
    group.plan()

    group.go()

    print "end POSE = ", pose

if __name__ == '__main__':
    main()


