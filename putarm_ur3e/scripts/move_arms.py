#!/usr/bin/env python

from math import pi, sqrt
from random import uniform

import rospy
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Pose

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from arm_planner import ArmPlanner
from arm_planner import Scene


class MoveArms(ArmPlanner):
    def __init__(self):
        super(ArmPlanner, self).__init__()
        self.ur3e_arm = ArmPlanner()
        #self.init_movements()
        #self.sequential_movements()

    def sequential_movements(self):
        """
        Definied sequential movements for left and right arm
        """
        movements = []

        px = 0.12
        py = 0.42
        pz = 0.12
        ex = -1.57
        ey = 0.0
        ez = 0.0

        # Left arm movements
        movements.append({"px": px, "py": py, "pz": pz,
                               "ex": ex, "ey": ey, "ez": ez})

        # px = -0.25
        # py = 0.25
        # pz = 0.50
        # ex = 0.0
        # ey = 0.0
        # ez = 0.0

        # # Left arm movements
        # movements.append({"px": px, "py": py, "pz": pz,
        #                        "ex": ex, "ey": ey, "ez": ez})


        # px = 0.25
        # py = 0.25
        # pz = 0.50
        # ex = 0.0
        # ey = 0.0
        # ez = 0.0

        # # Left arm movements
        # movements.append({"px": px, "py": py, "pz": pz,
        #                        "ex": ex, "ey": ey, "ez": ez})

        # # py -= 0.1
        # # px += 0.05
        # ex += 0.2
        # movements.append({"px": px, "py": py, "pz": pz,
        #                        "ex": ex, "ey": ey, "ez": ez})

        # # pz += 0.1
        # # py -= 0.05
        # ey += 0.1
        # movements.append({"px": px, "py": py, "pz": pz,
        #                        "ex": ex, "ey": ey, "ez": ez})

        # # pz -= 0.1
        # # px += 0.05
        # ez += 0.2
        # movements.append({"px": px, "py": py, "pz": pz,
        #                        "ex": ex, "ey": ey, "ez": ez})

        # ez -= 1.0
        # movements.append({"px": px, "py": py, "pz": pz,
        #                        "ex": ex, "ey": ey, "ez": ez})

        # # px += 0.1
        # # movements.append({"px": px, "py": py, "pz": pz,
        # #                        "ex": ex, "ey": ey, "ez": ez})

        arm_poses = self.sequential_poses_generator(movements)


        for key, value in enumerate(arm_poses):
            print(arm_poses[key])
            self.ur3e_arm.move_pose(arm_poses[key].position, arm_poses[key].orientation)
            rospy.sleep(1.0)

    def sequential_poses_generator(self, movements):
        """
        Get movements described as positions and euler angles and return positions and quaternions
        """
        arm_poses = []
        for movement in movements:
            quaternion = quaternion_from_euler(movement['ex'], movement['ey'], movement['ez'])
            pose = {'px': movement['px'], 'py': movement['py'], 'pz': movement['pz'],
                    'qx': quaternion[0], 'qy': quaternion[1], 'qz': quaternion[2], 'qw': quaternion[3]}
            # pose = {'px': movement['px'], 'py': movement['py'], 'pz': movement['pz'],
            #         'qx': -0, 'qy': 0.7, 'qz': 0.7, 'qw': 0}
            new_arm_pose = self.get_pose(pose)
            arm_poses.append(new_arm_pose)
        return arm_poses

    def init_movements(self):
        # Initial pose
        params = {"px": 0.15, "py": 0.15, "pz": 0.45,
                  "ex": 0.0, "ey": 0.0, "ez": 0.0}
        arm_poses = self.init_poses_generator(params)
        for arm_pose in arm_poses:
            print(arm_pose)
            self.ur3e_arm.move_pose(arm_pose.position, arm_pose.orientation)
            #self.right_arm.move_pose(arm_pose.position, arm_pose.orientation)
            rospy.sleep(0.5)

    def init_poses_generator(self, params, span_p=0.05, span_q=0.2):
        """
        Generate pose with random movement in a single direction through all parameters of translation and rotation
        """
        arm_poses = []
        for key in params:
            # Check the argument under consideration - translation or rotation, then assign proper randomize factor
            span = 0.0
            if key[0] == "p": # positions
                span = span_p
            elif key[0] == "e": # euler angles
                span = span_q

            # Get new arm pose with random movement in single specific direction
            movement = uniform(-span, span)
            params[key] += movement

            # Set new params list with position and orientation
            quaternion = quaternion_from_euler(params['ex'], params['ey'], params['ez'])
            pose={'px':params['px'], 'py':params['py'], 'pz':params['pz'],
                  'qx': quaternion[0], 'qy': quaternion[1], 'qz': quaternion[2], 'qw': quaternion[3]}
            new_arm_pose = self.get_pose(pose)
            arm_poses.append(new_arm_pose)

        return arm_poses

    def get_pose(self, pose):
        new_pose = Pose(position=Point(
                 x=pose['px'],
                 y=pose['py'],
                 z=pose['pz']),
             orientation=Quaternion(
                 x=pose['qx'],
                 y=pose['qy'],
                 z=pose['qz'],
                 w=pose['qw']))
        return new_pose

if __name__ == '__main__':
    rospy.init_node('move_arms_node')
    ma = MoveArms()
    rospy.spin()


# def quat2list(q):
#     return q.x, q.y, q.z, q.w

# rospy.init_node('move_arms')
# rospy.sleep(2.)
# left_arm = ArmPlanner("left_arm", "left_arm")
# right_arm = ArmPlanner("right_arm", "right_arm")

#i = 0
#L = 1.
#ds = 0.01
#dfi = 0.3
#
#scene = Scene()
#scene.add_box("blat", Point(0.1, 0., -0.01), Quaternion(0., 0., 0., 1.), (1.0, 2.0, 0.02))
#scene.add_box("sciana", Point(-0.2, 0., 0.3), Quaternion(0., 0., 0., 1.), (0.02, 2.0, 1.5))
#
#left_arm.add_box("l_chwytak", Point(0.05, 0., 0.), Quaternion(0., 0., 0., 1.), (0.1, 0.1, 0.1))
#right_arm.add_box("r_chwytak", Point(0.05, 0., 0.), Quaternion(0., 0., 0., 1.), (0.1, 0.1, 0.1))

#left = True
#while not rospy.is_shutdown():
#    if left:
#        moving_arm = left_arm
#        static_arm = right_arm
#        y_b, y_t = 0.1, 0.6
#    else:
#        moving_arm = right_arm
#        static_arm = left_arm
#        y_b, y_t = -0.6, -0.1
#    m_pose = moving_arm.group.get_current_pose(moving_arm.eef_link).pose
#    s_pose = static_arm.group.get_current_pose(static_arm.eef_link).pose
#
#    m_pos = m_pose.position
#    m_ort = euler_from_quaternion(quat2list(m_pose.orientation))
#
#    s_pos = s_pose.position
#    s_ort = euler_from_quaternion(quat2list(s_pose.orientation))
#
#    dist = 10.
#    while dist > L:
#        new_x = np.clip(m_pos.x + ds * (2 * random() - 1), -0.1, 0.5)
#        new_y = np.clip(m_pos.y + ds * (2 * random() - 1), y_b, y_t)
#        new_z = np.clip(m_pos.z + ds * (2 * random() - 1), 0.2, 0.5)
#        new_ort = m_ort + dfi * (2 * np.random.rand(3) - 1)
#        print(new_ort)
#
#        dist = sqrt((new_x - s_pos.x)**2 + (new_y - s_pos.y)**2 + (new_z - s_pos.z)**2)
#
#    n_pos = Point(new_x, new_y, new_z)
#    n_ort = Quaternion(*quaternion_from_euler(*new_ort))
#    moving_arm.move_pose(n_pos, n_ort)
#    left = not left





# position = Point(0.22, 0.15, 0.47)
# orientation = Quaternion(0., 0., 0., 1.)
# left_arm.move_pose(position, orientation)
# position = Point(0.25, 0.15, 0.47)
# orientation = Quaternion(0., 0., 0., 1.)
# right_arm.move_pose(position, orientation)
##position = Point(0.2, 0.0, 0.5)

##a = list(quaternion_from_euler(0., 0., pi/2))
##orientation = Quaternion(*a)
#position = Point(0.0, 0.0, 0.5)
#orientation = Quaternion(0., 0., 0., 1.)
#right_arm.move_pose(position, orientation)

#left_arm.add_box("xD", Point(0., 0., 0.3), Quaternion(0., 0., 0., 1.), (0.2, 2.0, 0.6))
#right_arm.add_box("xD", Point(0., 0., 0.3), Quaternion(0., 0., 0., 1.), (0.2, 2.0, 0.6))
#
#z = 1.0
#y = 0.3
#positions_l = [Point(0.4, y, z + 0.1), Point(0.4, 0.2, z), Point(0.3, y + 0.1, z)]
#orientations_l = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.), Quaternion(0., 0., -sqrt(2)/2, sqrt(2)/2)]
#orientations_l = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.)]
#left_arm.move_path(positions_l, orientations_l)
#positions_r = [Point(0.4, -y, z + 0.1), Point(0.4, -y, z), Point(0.3, -y - 0.1, z)]
##orientations_r = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., sqrt(2)/2, sqrt(2)/2), Quaternion(0., 0., 0., 1.)]
##orientations_r = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.)]
#orientations_r = [Quaternion(0., 0., 0., 1.), Quaternion(0., 0., 0., 1.), Quaternion(0., 0., sqrt(2)/2, sqrt(2)/2)]
#right_arm.move_path(positions_r, orientations_r)
