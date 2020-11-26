#!/usr/bin/env python2
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from math import pi
from moveit_commander.conversions import pose_to_list


class Scene(object):
    """Scene"""

    def __init__(self, ns=""):
        super(Scene, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface(ns)

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
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position = position
        pose.pose.orientation = orientation
        print(self.scene.add_box(name, pose, size=size))
        self.last_object_name = name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_object(self, name, filename, position, orientation, size=(1., 1., 1.), timeout=4):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position = position
        pose.pose.orientation = orientation
        print(self.scene.add_mesh(name, pose, filename, size=size))
        self.last_object_name = name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def remove_object(self, name, timeout=4):
        self.scene.remove_world_object(name)
        return self.wait_for_state_update(box_is_known=False, timeout=timeout)
