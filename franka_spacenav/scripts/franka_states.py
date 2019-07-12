#!/usr/bin/python

import rospy
import tf.transformations
import numpy as np

from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped

class Franka(object):
    def __init__(self):
        self.position = None
        self.quaternion = None

        self.position_d = None
        self.quaternion_d = None

        self.state_sub = rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
        self.desired_state_sub = rospy.Subscriber("/equilibrium_pose", PoseStamped, self.desired_state_callback)

    def franka_state_callback(self, msg):
        quaternion = tf.transformations.quaternion_from_matrix(np.transpose(np.reshape(msg.O_T_EE,(4, 4))))
        self.quaternion = quaternion / np.linalg.norm(quaternion)    # normalize the quaternion
        position_x = msg.O_T_EE[12]
        position_y = msg.O_T_EE[13]
        position_z = msg.O_T_EE[14]
        self.position = [position_x, position_y, position_z]

    def franka_pose(self):
        if (self.position or self.quaternion) is None:
            rospy.logwarn("\nFranka pose haven't been filled yet.")
            return None
        else:
            return self.position, self.quaternion

    def desired_state_callback(self, msg):
        self.position_d = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.quaternion_d = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    def franka_pose_desired(self):
        if (self.position_d or self.quaternion_d) is None:
            rospy.logwarn("\nFranka pose desired haven't been filled yet.")
            return None
        else:
            return self.position_d, self.quaternion_d

if __name__ == "__main__":
    rospy.init_node("franka_states_node")

    robot = Franka()
    rospy.sleep(1)

    # ==========================================================================
    # KST FILE
    # ==========================================================================
    path = '/home/ochoa/kst/franka_test.txt'
    file = open(path, "w")
    line1 = [' t; ', 'pX; ', 'pXd; ', 'pY; ', 'pYd; ', 'pZ; ', 'pZd; ',
                     'QX; ', 'QXd; ', 'QY; ', 'QYd; ', 'QZ; ', 'QZd; ', 'QW; ', 'QWd; ',
                     'Yaw(X); ', 'Yaw_d(Xd); ', 'Pitch(Y); ', 'Pitch_d(Yd); ', 'Roll(Z); ', 'Roll(Zd) ', '\n']
    line2 = [' s; ', 'm; ', 'm; ', 'm; ', 'm; ', 'm; ', 'm; ',
                     'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ', 'q; ',
                     'rad; ', 'rad; ', 'rad; ', 'rad; ', 'rad; ', 'rad ', '\n']
    file.writelines(line1)
    file.writelines(line2)

    # ==========================================================================
    # main loop
    # ==========================================================================
    count = 0
    rate = rospy.Rate(1000) # 1khz
    while not rospy.is_shutdown():
        count = count + 1

        position, quaternion = robot.franka_pose()
        # print '\nposition: ', position
        # print '\nquaternion: ', quaternion
        euler_angles = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')

        position_d, quaternion_d = robot.franka_pose_desired()
        # print '\nposition_d: ', position_d
        # print '\nquaternion_d: ', quaternion_d

        euler_angles_d = tf.transformations.euler_from_quaternion(quaternion_d, axes='sxyz')

        # =============================================================================================
        # SEND TO FILE
        # =============================================================================================
        TIME = count/1000.0
        # position
        pX = position[0]
        pXd =position_d[0]
        pY = position[1]
        pYd = position_d[1]
        pZ = position[2]
        pZd = position_d[2]

        # orientation
        QX = quaternion[0]
        QXd = quaternion_d[0]
        QY = quaternion[1]
        QYd = quaternion_d[1]
        QZ = quaternion[2]
        QZd = quaternion_d[2]
        QW = quaternion[3]
        QWd = quaternion_d[3]

        # orientation in euler angles
        Yaw = euler_angles[0]
        Yaw_d = euler_angles_d[0]
        Pitch = euler_angles[1]
        Pitch_d = euler_angles_d[1]
        Roll = euler_angles[2]
        Roll_d = euler_angles_d[2]


        lines = ' ' + str(TIME) + '; ' \
                + str(pX) + '; ' + str(pXd) + '; ' \
                + str(pY) + '; ' + str(pYd) + '; ' \
                + str(pZ) + '; ' + str(pZd) + '; ' \
                + str(QX) + '; ' + str(QXd) + '; ' \
                + str(QY) + '; ' + str(QYd) + '; ' \
                + str(QZ) + '; ' + str(QZd) + '; ' \
                + str(QW) + '; ' + str(QWd) + '; ' \
                + str(Yaw) + '; ' + str(Yaw_d) + '; ' \
                + str(Pitch) + '; ' + str(Pitch_d) + '; ' \
                + str(Roll) + '; ' + str(Roll_d)
        file.write(lines + '\n')

        rate.sleep()

    file.close()
