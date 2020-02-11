#!/usr/bin/python

import rospy
import numpy as np
import sys

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

robot_state_flag = False

class udrilling_state(object):
    def __init__(self):
        self.position = None
        self.orientation = None

        self.position_d = None
        self.orientation_d = None

        self.error = None

        self.ee_wrench = None
        
        # subscribers
        self.pose_sub = rospy.Subscriber("/robot_poseEE", PoseStamped, self.pose_subCallback)
        self.pose_d_sub = rospy.Subscriber("/robot_poseEE_d", PoseStamped, self.pose_d_subCallback)
        self.error_sub = rospy.Subscriber("/robot_error", Twist, self.error_subCallback)
        self.ee_wrench_sub = rospy.Subscriber("/robot_wrench", Twist, self.ee_wrench_subCallback)


    def pose_subCallback(self, msg):
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        global robot_state_flag
        robot_state_flag = True

    def robot_pose(self):
        if (self.position or self.orientation) is None:
            rospy.logwarn("\nRobot pose haven't been filled yet.")
            return None
        else:
            return self.position, self.orientation

    def pose_d_subCallback(self, msg):
        self.position_d = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.orientation_d = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    def robot_pose_d(self):
        if (self.position_d or self.orientation_d) is None:
            rospy.logwarn("\nRobot desired pose haven't been filled yet.")
            return None
        else:
            return self.position_d, self.orientation_d

    def error_subCallback(self, msg):
        self.error = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]

    def robot_error(self):
        if (self.error) is None:
            rospy.logwarn("\nRobot error haven't been filled yet.")
            return None
        else:
            return self.error

    def ee_wrench_subCallback(self, msg):
        self.ee_wrench = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]

    def robot_wrench(self):
        if (self.ee_wrench) is None:
            rospy.logwarn("\nRobot wrench haven't been filled yet.")
            return None
        else:
            return self.ee_wrench

if __name__ == "__main__":
    rospy.init_node("udrilling_controller_data")

    robot = udrilling_state()
    rate = rospy.Rate(1000) # 1000 Hz
    while not robot_state_flag:
        rospy.sleep(1)
        rate.sleep()
  
    # ==========================================================================
    # KST FILE
    # ==========================================================================
    path = '/home/panda/kst/udrilling/udrilling_controller'
    file = open(path, "w")
    line1 = ['t ',
             'px ', 'py ', 'pz ',
             'pxd ', 'pyd ', 'pzd ',
             'ox ', 'oy ', 'oz ', 'ow ',
             'oxd ', 'oyd ', 'ozd ', 'owd ',
             'epx ', 'epy ', 'epz ', 'eox ', 'eoy ', 'eoz ',
             'Fx ', 'Fy ', 'Fz ', '\n']
    line2 = ['s ',
             'm ', 'm ', 'm ',
             'm ', 'm ', 'm ',
             'unitQ ', 'unitQ ', 'unitQ ', 'unitQ ',
             'unitQ ', 'unitQ ', 'unitQ ', 'unitQ ',
             'm ', 'm ', 'm ', 'rad ', 'rad ', 'rad ',
             'N ', 'N ', 'N ', '\n']
    file.writelines(line1)
    file.writelines(line2)

    # ==========================================================================
    # main loop
    # ==========================================================================
    count = 0
    while not rospy.is_shutdown():
        count = count + 1

        position, orientation = robot.robot_pose()
        # print '\nposition: ', position
        # print '\norientation: ', orientation

        position_d, orientation_d = robot.robot_pose_d()
        # print '\nposition_d: ', position_d
        # print '\norientation_d: ', orientation_d

        error = robot.robot_error()
        # print '\nerror: ', error

        ee_wrench = robot.robot_wrench()
        # print '\nee_wrench: ', ee_wrench

        # =============================================================================================
        # SEND TO FILE
        # =============================================================================================
        TIME = count * 0.001
        px = position[0]
        py = position[1]
        pz = position[2]

        pxd = position_d[0]
        pyd = position_d[1]
        pzd = position_d[2]

        ox = orientation[0]
        oy = orientation[1]
        oz = orientation[2]
        ow = orientation[3]

        oxd = orientation_d[0]
        oyd = orientation_d[1]
        ozd = orientation_d[2]
        owd = orientation_d[3]

        epx = error[0]
        epy = error[1]
        epz = error[2]
        eox = error[3]
        eoy = error[4]
        eoz = error[5]

        Fx = ee_wrench[0]
        Fy = ee_wrench[1]
        Fz = ee_wrench[2]

        lines = str(TIME) + ' ' \
                + str(px) + ' ' + str(py) + ' ' + str(pz) + ' ' + str(pxd) + ' ' + str(pyd) + ' ' + str(pzd) + ' ' \
                + str(ox) + ' ' + str(oy) + ' ' + str(oz) + ' ' + str(ow) + ' ' + str(oxd) + ' ' + str(oyd) + ' ' + str(ozd) + ' ' + str(owd) + ' ' \
                + str(epx) + ' ' + str(epy) + ' ' + str(epz) + ' ' + str(eox) + ' ' + str(eoy) + ' ' + str(eoz) + ' ' \
                + str(Fx) + ' ' + str(Fy) + ' ' + str(Fz)
        file.write(lines + '\n')
        
        #f"{a:.3f}"
        #f.write(f"")             
        
        rate.sleep()

    file.close()

    