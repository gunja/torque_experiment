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
    group = moveit_commander.MoveGroupCommander( "manipulator")
    #group = moveit_commander.MoveGroupCommander( "base")
    group.allow_replanning( True)
    group.set_planning_time( 10.0)
    #group.set_goal_joint_tolerance( 0.001)

    #group.set_goal_position_tolerance( 0.001)
    #group.set_goal_orientation_tolerance( 0.001)


    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = JOINT_NAMES

    for i in range(5):
        print "Starting step #", i + 1, " of 5"

        for j in [ 
            [ 0.354, 0.842, 0.439, 0.558, 0.448, 0.453, 0.532 ]
            ,  [ -0.019, 1.147, 0.618, -0.690, -0.022, 0.717, -0.096 ]
            , [ -0.038, 0.864, 0.488, -0.693, -0.044, 0.708, -0.128 ]
          ]:
            print "Target is ", j
            group.clear_pose_targets()
            group.set_start_state_to_current_state()
            pose = group.get_current_pose().pose
            target_pose = copy.deepcopy(pose)

            target_pose.position.x = j[0]
            target_pose.position.y = j[1]
            target_pose.position.z = j[2]
            target_pose.orientation.x = j[3]
            target_pose.orientation.y = j[4]
            target_pose.orientation.z = j[5]
            target_pose.orientation.w = j[6]
            

            joints = group.get_current_joint_values()

            group.set_pose_target(target_pose)
            plan = group.plan()

            print "final point=", plan.joint_trajectory.points[len(plan.joint_trajectory.points) -1]

            goal.trajectory = plan.joint_trajectory
            client.send_goal( goal)

            client.wait_for_result()
            res = client.get_result()

            print("res=", res)
            pose = group.get_current_pose().pose
            print "end POSE = ", pose
            r= raw_input( "Enter something for next step")

if __name__ == '__main__':
    main()
