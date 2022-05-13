#!/usr/bin/env python

import sys
import copy
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.group_name = "welding_arm"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
   

  def go_to_joint_state(self):

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -0.42
    joint_goal[2] = 0.68
    joint_goal[3] = 0
    joint_goal[4] = -0.64
    joint_goal[5] = 0

    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()
 
    scale = 1
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    print wpose
    wpose.position.x += scale * 0.1
    wpose.position.x += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    (plan ,fraction) = self.move_group.compute_cartesian_path(waypoints,0.01,0.0)
    self.move_group.execute(plan, wait=True)

    self.move_group.set_named_target("Home")
    plan = self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()


    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()
    tutorial.go_to_joint_state()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

