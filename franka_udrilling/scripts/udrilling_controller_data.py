#!/usr/bin/python

import rospy
import numpy as np
import array as arr
from matplotlib import pyplot
from scipy import signal

import tf.transformations
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped

class Franka(object):
    def __init__(self):
        self.position = None
        self.orientation = None

        self.position_d = None
        self.orientation_d = None

        self.EE_force = None

        self.state_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
        self.desired_state_sub = rospy.Subscriber("/panda_equilibrium_pose", PoseStamped, self.desired_state_callback)

    def franka_state_callback(self, msg):
        
        orientation = tf.transformations.quaternion_from_matrix(np.transpose(np.reshape(msg.O_T_EE,(4, 4))))
        self.orientation = orientation / np.linalg.norm(orientation)    # normalize the quaternion
        
        position_x = msg.O_T_EE[12]
        position_y = msg.O_T_EE[13]
        position_z = msg.O_T_EE[14]
        self.position = [position_x, position_y, position_z]

        self.EE_force = [msg.K_F_ext_hat_K[0], msg.K_F_ext_hat_K[1], msg.K_F_ext_hat_K[2]]

    def franka_pose(self):
        if (self.position or self.orientation) is None:
            rospy.logwarn("\nFranka pose haven't been filled yet.")
            return None
        else:
            return self.position, self.orientation

    def franka_EE_force(self):
        if (self.EE_force) is None:
            rospy.logwarn("\nFranka End-Effector haven't been filled yet.")
            return None
        else:
            return self.EE_force

    def desired_state_callback(self, msg):
        self.position_d = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.orientation_d = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    def franka_pose_desired(self):
        if (self.position_d or self.orientation_d) is None:
            rospy.logwarn("\nFranka pose desired haven't been filled yet.")
            return None
        else:
            return self.position_d, self.orientation_d



if __name__ == "__main__":
    rospy.init_node("franka_states_node")

    robot = Franka()
    rospy.sleep(1)

    

    # ==========================================================================
    # main loop
    # ==========================================================================
    
    # plot init values    
    t = []
    px = []
    py = []
    pz = []
    pxd = []
    pyd = []
    pzd = []
    Fx = []
    Fy = []
    Fz = []


    count = 0
    rate = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        count = count + 1

        position, orientation = robot.franka_pose()
        # print '\nposition: ', position
        # print '\norientation: ', orientation

        position_d, orientation_d = robot.franka_pose_desired()
        # print '\nposition_d: ', position_d
        # print '\norientation_d: ', orientation_d

        EE_force = robot.franka_EE_force()
         # print '\nEE_force: ', EE_force

        
        # plot
        t.append(count/100.0)
        
        px.append(position[0])
        py.append(position[1])
        pz.append(position[2])

        pxd.append(position_d[0])
        pyd.append(position_d[1])
        pzd.append(position_d[2])

        Fx.append(EE_force[0])
        Fy.append(EE_force[1])
        Fz.append(EE_force[2])

        # -----------------------------------------------------------------------
        pyplot.figure(1) 
        pyplot.subplot(3, 1, 1)
        pyplot.plot(t, px, '--r', linewidth = 1.5, label='robot')
        pyplot.plot(t, pxd, 'k', linewidth = 1.5, label='desired')   
        pyplot.grid(True)
        pyplot.xlabel('t(s)')
        pyplot.ylabel('px(m)')

        pyplot.subplot(3, 1, 2)
        pyplot.plot(t, py, '--g', linewidth = 1.5, label='robot')
        pyplot.plot(t, pyd, 'y', linewidth = 1.5, label='desired')    
        pyplot.grid(True)
        pyplot.xlabel('t(s)')
        pyplot.ylabel('py(m)')

        pyplot.subplot(3, 1, 3)
        pyplot.plot(t, pz, '--b', linewidth = 1.5, label='robot')
        pyplot.plot(t, pzd, 'm', linewidth = 1.5, label='desired')    
        pyplot.grid(True)
        pyplot.xlabel('t(s)')
        pyplot.ylabel('pz(m)')

        # -----------------------------------------------------------------------
        pyplot.figure(2)
        pyplot.subplot(3, 1, 1)
        pyplot.plot(t, Fx, 'r', linewidth = 1.5)
        pyplot.grid(True)
        pyplot.xlabel('t(s)')
        pyplot.ylabel('Fx(N)')

        pyplot.subplot(3, 1, 2)
        pyplot.plot(t, Fy, 'g', linewidth = 1.5)
        pyplot.grid(True)
        pyplot.xlabel('t(s)')
        pyplot.ylabel('Fy(N)')

        pyplot.subplot(3, 1, 3)
        pyplot.plot(t, Fz, 'b', linewidth = 1.5)
        pyplot.grid(True)
        pyplot.xlabel('t(s)')
        pyplot.ylabel('Fz(N)')
        # -----------------------------------------------------------------------

        pyplot.pause(0.01)
                      
        rate.sleep()

    pyplot.show()

    