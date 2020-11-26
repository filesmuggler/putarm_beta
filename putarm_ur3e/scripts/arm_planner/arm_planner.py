#!/usr/bin/env python2
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from math import pi
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

  elif type(goal) is PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class ArmPlanner(object):
  """Arm Planner"""
  def __init__(self,ns=''):
    super(ArmPlanner, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    #print(robot_description)
    robot = moveit_commander.RobotCommander()

    print "============ Robot Groups:", robot.get_group_names()

    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_planning_time(10.)
    
    #group.set_planner_id("RRTstarkConfigDefault")

    display_trajectory_publisher = rospy.Publisher(ns + '/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    #print robot.get_current_state()
    print group.get_current_pose()
    print ""

    self.robot = robot
    self.scene = scene
    self.group = group
    self.group_name = group_name
    self.ns = ns
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.last_object_name = None

  def move_joints(self, goal):
    """
    Moves robot to specified joints positions
    :param goal: list[6]
    :return: Boolean whether goal is reached within tolerance
    """
    self.group.go(goal, wait=True)
    self.group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(goal, current_joints, 0.01)

  def move_pose(self, position, orientation):
    """
    Moves robot to specified position and orientation in the world coordinate system
    :param position: Point()
    :param orientation: Quaternion()
    :return: Boolean whether goal is reached within tolerance
    """
    pose_goal = Pose()
    pose_goal.orientation = orientation
    pose_goal.position = position
    self.group.set_pose_target(pose_goal)

    self.group.go(wait=True)
    self.group.stop()
    self.group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def move_path(self, positions, orientations):
    """
    Moves robot to list of specified positions and orientations in the world coordinate system
    :param positions: list of Points
    :param orientations: list of Quaternions
    :return: Boolean whether last pose is reached within tolerance
    """
    waypoints = []
    assert len(positions) == len(orientations)
    for i in range(len(positions)):
      pose_goal = Pose()
      pose_goal.position = positions[i]
      pose_goal.orientation = orientations[i]
      waypoints.append(pose_goal)

    #print("WAYPOINTS", waypoints)

    (plan, fraction) = self.group.compute_cartesian_path(waypoints,   # waypoints to follow
                                                         0.01,        # eef_step
                                                         0.0)         # jump_threshold
    print("PLAN:", plan)
    print("FRAC:", fraction)

    #display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #display_trajectory.trajectory_start = self.robot.get_current_state()
    #display_trajectory.trajectory.append(plan)
    #self.display_trajectory_publisher.publish(display_trajectory)

    self.group.execute(plan, wait=True)
    #self.group.execute(plan, wait=False)

    #pose_goal = Pose()
    #pose_goal.orientation = orientations[-1]
    #pose_goal.position = positions[-1]
    #current_pose = self.group.get_current_pose().pose
    #return all_close(pose_goal, current_pose, 0.01)
    return True

  def wait_for_state_update(self, box_is_known=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      is_known = self.last_object_name in self.scene.get_known_object_names()
      if box_is_known == is_known:
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

  def add_box(self, name, position, orientation, size, timeout=4):
    eef_link = self.group_name + "_ee_link"
    pose = PoseStamped()
    pose.header.frame_id = eef_link
    pose.pose.position = position
    pose.pose.orientation = orientation
    self.scene.add_box(name, pose, size=size)
    self.scene.attach_box(eef_link, name, touch_links=[eef_link])
    self.last_object_name = name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def add_object(self, name, filename, position, orientation, size=(1., 1., 1.), timeout=4):
    pose = PoseStamped()
    pose.header.frame_id = self.group_name + "_ee_link"
    pose.pose.position = position
    pose.pose.orientation = orientation
    print(self.scene.add_mesh(name, pose, filename, size=size))
    self.last_object_name = name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def remove_object(self, name, timeout=4):
    self.scene.remove_world_object(name)
    return self.wait_for_state_update(box_is_known=False, timeout=timeout)

