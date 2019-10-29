// =============================================================================
// Name        : udrilling_state.h
// Author      : Hélio Ochoa
// Description : Displays the micro-drilling state (robot pose, external forces)
//               and publishes the created references in udrilling_controller...
// =============================================================================
#ifndef UDRILLING_STATE_H
#define UDRILLING_STATE_H

#include <sstream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <SFML/Window/Keyboard.hpp>
#include <Eigen/Dense>

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_msgs/FrankaState.h>

#define PI 3.14159265358979323846 
#define CLEANWINDOW "\e[2J\e[H"

namespace franka_udrilling {
    
    class uDrillingState{
        public:
            uDrillingState(ros::NodeHandle &nh);
            ~uDrillingState();
    
            // PUBLIC VARIABLES
            Eigen::Matrix4d O_T_EE; // robot transformation matrix
            Eigen::Matrix<double, 6, 1> O_F_ext_hat_K; // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame
            Eigen::Matrix<double, 6, 1> K_F_ext_hat_K; // estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame

            int spacenav_button_1 = 0;
            int spacenav_button_2 = 0;

            ros::Publisher pose_pub; // pose publisher

            // PUBLIC FUNCTIONS
            Eigen::Vector3d polynomial3Trajectory(Eigen::Vector3d& pi, Eigen::Vector3d& pf, double ti, double tf, double t);
            Eigen::VectorXd robotPose(Eigen::Matrix4d& Xd);  // Function to get the current robot pose  
            void posePublisherCallback(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);
      
        private:
            // PRIVATE VARIABLES
            ros::NodeHandle nh_; // node handle
            ros::Subscriber spacenav_sub; // spacenav subscriber
            ros::Subscriber franka_state_sub; // franka state subscriber
            int franka_state_flag = 0;

            // PRIVATE FUNCTIONS
            void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
            void frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg);

  };

} // namespace franka_udrilling


#endif // UDRILLING_STATE_H