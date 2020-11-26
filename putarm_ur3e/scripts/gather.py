#!/usr/bin/env python
import os
from math import pi, sqrt
from random import random

import rospy as rp
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from arm_planner import ArmPlanner
from arm_planner import Scene


def quat2list(q):
    return q.x, q.y, q.z, q.w


def pos2list(q):
    return q.x, q.y, q.z


class Dataset:
    def __init__(self):
        self.img_1 = None
        self.img_2 = None
        rp.init_node('dataset_creator', anonymous=True)
        rp.Subscriber("/logitech/image_raw/", Image, self.img_1_callback)
        rp.Subscriber("/realsense/image_raw/", Image, self.img_2_callback)
        self.right_arm = ArmPlanner("right_arm")
        self.left_arm = ArmPlanner("left_arm")
        self.scene = Scene()

        self.i = 0
        self.L = 0.55
        self.ds = 0.03
        #self.dfi = 0.1
        self.dfi = np.array([.15, 0.07, .05])

        self.add_constraints()

    def add_constraints(self):
        self.scene.add_box("blat", Point(0.1, 0., -0.01), Quaternion(0., 0., 0., 1.), (1.0, 2.0, 0.02))
        self.scene.add_box("sciana", Point(-0.2, 0., 0.3), Quaternion(0., 0., 0., 1.), (0.02, 2.0, 1.5))
        self.left_arm.add_box("l_chwytak", Point(0.08, 0., 0.), Quaternion(0., 0., 0., 1.), (0.15, 0.07, 0.07))
        self.right_arm.add_box("r_chwytak", Point(0.08, 0., 0.), Quaternion(0., 0., 0., 1.), (0.15, 0.07, 0.07))
        self.scene.add_box("sztuczny_sufit", Point(0.1, 0., 0.61), Quaternion(0., 0., 0., 1.), (1.0, 2.0, 0.02))

    def img_1_callback(self, data):
        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.img_1 = im

    def img_2_callback(self, data):
        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.img_2 = im

    def save(self):
        print("IMG 1:", self.img_1.shape)
        print("IMG 2:", self.img_2.shape)
        l_pos = pos2list(self.left_arm.group.get_current_pose(self.left_arm.eef_link).pose.position)
        l_ort = euler_from_quaternion(
            quat2list(self.left_arm.group.get_current_pose(self.left_arm.eef_link).pose.orientation))
        r_pos = pos2list(self.right_arm.group.get_current_pose(self.right_arm.eef_link).pose.position)
        r_ort = euler_from_quaternion(
            quat2list(self.right_arm.group.get_current_pose(self.right_arm.eef_link).pose.orientation))
        conf = np.array([l_pos, l_ort, r_pos, r_ort])
        print(conf.shape)

        path = os.path.dirname(__file__)
        plt.imsave(path + "/data/cam_1_" + str(self.i).zfill(6) + ".png", self.img_1)
        plt.imsave(path + "/data/cam_2_" + str(self.i).zfill(6) + ".png", self.img_2)
        np.savetxt(path + "/data/arms_" + str(self.i).zfill(6) + ".conf", conf)
        self.i += 1

    def loop(self):
        left = True
        while not rp.is_shutdown():
            if left:
                moving_arm = self.left_arm
                static_arm = self.right_arm
                y_b, y_t = 0.0, 0.6
            else:
                moving_arm = self.right_arm
                static_arm = self.left_arm
                y_b, y_t = -0.6, -0.0
            m_pose = moving_arm.group.get_current_pose(moving_arm.eef_link).pose
            s_pose = static_arm.group.get_current_pose(static_arm.eef_link).pose

            m_pos = m_pose.position
            m_ort = euler_from_quaternion(quat2list(m_pose.orientation))

            s_pos = s_pose.position
            s_ort = euler_from_quaternion(quat2list(s_pose.orientation))

            dist = 10. * self.L
            while dist > self.L:
                new_x = np.clip(m_pos.x + self.ds * (2 * random() - 1), -0.1, 0.5)
                new_y = np.clip(m_pos.y + self.ds * (2 * random() - 1), y_b, y_t)
                new_z = np.clip(m_pos.z + self.ds * (2 * random() - 1), 0.2, 0.5)
                new_ort = m_ort + self.dfi * (2 * np.random.rand(3) - 1)
                print(new_ort)

                dist = sqrt((new_x - s_pos.x) ** 2 + (new_y - s_pos.y) ** 2 + (new_z - s_pos.z) ** 2)

            #n_pos = Point(new_x, new_y, new_z)
            #n_ort = Quaternion(*quaternion_from_euler(*new_ort))
            #moving_arm.move_pose(n_pos, n_ort)

            print("NEW STATE")
            print(new_ort)
            print(new_x)
            print(new_y)
            print(new_z)

            N = 4
            path_pos = []
            path_ort = []
            for i in range(N):
                x = float(N - 1 - i) / N * m_pos.x + float(i + 1) / N * new_x
                y = float(N - 1 - i) / N * m_pos.y + float(i + 1) / N * new_y
                z = float(N - 1 - i) / N * m_pos.z + float(i + 1) / N * new_z
                #print(new_ort)
                #print(m_ort)
                ort = float(N - 1 - i) / N * np.array(m_ort) + float(i + 1) / N * new_ort

                n_pos = Point(x, y, z)
                n_ort = Quaternion(*quaternion_from_euler(*ort))

                path_pos.append(n_pos)
                path_ort.append(n_ort)


            moving_arm.move_path(path_pos, path_ort)
            #moving_arm.move_pose(n_pos, m_pose.orientation)
            left = not left

            rp.sleep(1.0)
            ds.save()
            rp.sleep(0.2)


if __name__ == '__main__':
    ds = Dataset()
    rp.sleep(1)
    ds.loop()
