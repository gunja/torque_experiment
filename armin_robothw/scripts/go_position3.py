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

    group.set_goal_position_tolerance( 0.001)
    group.set_goal_orientation_tolerance( 0.001)

    pose = group.get_current_pose().pose
  x: -0.723443259256
  y: -0.00402627531556
  z: -0.0295279067885
  w: 0.689740344235



    pose.position.x = -0.448716640732
    pose.position.y = 0.880891546971
    pose.position.z = 0.622184910876

    pose.orientation.x = -0.723443259256
    pose.orientation.y = -0.00402627531556
    pose.orientation.z =  -0.0295279067885
    pose.orientation.w = 0.689740344235

    group.set_pose_target(pose);
    group.plan()

    group.go()

    print "end POSE = ", pose

if __name__ == '__main__':
    main()


