#!/usr/bin/env python
import os
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np


path = os.path.dirname(os.path.abspath(__file__)) + '/../paths/'

joint_goal_astar = np.loadtxt(path +os.listdir(path)[0])
joint_goal_rrt = np.loadtxt(path +  os.listdir(path)[1])
joint_goal_potential = np.loadtxt(path +  os.listdir(path)[2])
joint_goal_sst = np.loadtxt(path +  os.listdir(path)[3])


def all_close(goal, actual, tolerance):

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


class MoveRobot(object):

  def __init__(self):


    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('lynx_move_group', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=2)


    planning_frame = move_group.get_planning_frame()


    #robot.get_current_state()

    # variables
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame


  def go_to_joint_state(self, joint_goal):
    move_group = self.move_group

    move_group.go(joint_goal[0:5], wait=True)

    current_joints = move_group.get_current_joint_values()


    rospy.loginfo("The current joints state is:")
    print(joint_goal)


    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, pose_value):

    move_group = self.move_group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose_value[0]
    pose_goal.position.y = pose_value[1]
    pose_goal.position.z = pose_value[2]
    pose_goal.orientation.w = pose_value[3]

    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose


    
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):

    move_group = self.move_group


    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))


    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction



  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)


    display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan):

    move_group = self.move_group
    move_group.execute(plan, wait=True)



def perform_planning(lynx_moving, joint_goal, method_name):

  joint_goal = joint_goal_astar
  for i in range(len(joint_goal)):
    lynx_moving.go_to_joint_state(joint_goal[i])


  cartesian_plan, fraction = lynx_moving.plan_cartesian_path()
  #lynx_moving.display_trajectory(cartesian_plan)
  lynx_moving.execute_plan(cartesian_plan)
  rospy.loginfo("%s Planning is Finished !", method_name)

def main():
  try:
    lynx_moving = MoveRobot()

    perform_planning(lynx_moving, joint_goal_astar, "Astar")
    perform_planning(lynx_moving, joint_goal_rrt, "RRT")
    perform_planning(lynx_moving, joint_goal_potential, "Potential Field")
    perform_planning(lynx_moving, joint_goal_sst, "SST")


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
