#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import argparse
import numpy as np


def moveJoint (jointcmds):
    topic_name = '/panda_arm_joint_trajectory_controller/command'
    pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
    jointCmd = JointTrajectory()
    point = JointTrajectoryPoint()
    jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    point.time_from_start = rospy.Duration.from_sec(5.0)
    for i in range(0, 7):
        jointCmd.joint_names.append('panda_joint'+str(i+1))
        point.positions.append(jointcmds[i])
        point.velocities.append(0)
        point.accelerations.append(0)
        point.effort.append(0)
    jointCmd.points.append(point)
    rate = rospy.Rate(100)
    count = 0
    while (count < 500):
        pub.publish(jointCmd)
        count = count + 1
        rate.sleep()

def moveFingers (jointcmds):
    topic_name = '/panda_hand_joint_trajectory_controller/command'
    pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
    jointCmd = JointTrajectory()
    point = JointTrajectoryPoint()
    jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
    point.time_from_start = rospy.Duration.from_sec(5.0)
    for i in range(0, 2):
        jointCmd.joint_names.append('panda_finger_joint'+str(i+1))
        point.positions.append(jointcmds[i])
        point.velocities.append(0)
        point.accelerations.append(0)
        point.effort.append(0)
    jointCmd.points.append(point)
    rate = rospy.Rate(100)
    count = 0
    while (count < 100):
        pub.publish(jointCmd)
        count = count + 1
        rate.sleep()


if __name__ == '__main__':
  try:
    rospy.init_node('move_to_start')
    #allow gazebo to launch
    rospy.sleep(1)

    #move robot to start position
    moveJoint ([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    moveFingers ([0,0])


  except rospy.ROSInterruptException:
    print "program interrupted before completion"
