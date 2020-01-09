// =============================================================================
// Name        : spacenav.h
// Author      : Hélio Ochoa
// Description : Allows user to control End-Effector(EE) pose in EE and Base(O)
//               frame with space-mouse.
// =============================================================================
#ifndef SPACENAV_H
#define SPACENAV_H

#include "ros/ros.h"
#include <sstream>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <SFML/Window/Keyboard.hpp>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>


#define PI  3.14159265358979323846  /* pi */
#define CLEANWINDOW "\e[2J\e[H"

namespace franka_simulation {

class Spacenav{   
    public:
        Spacenav(ros::NodeHandle &nh);
        ~Spacenav();

        // PUBLIC VARIABLES
        Eigen::Matrix<double, 6, 1> spacenav_motion;

        Eigen::Vector3d position_d;  // end-effector desired position
        Eigen::Quaterniond orientation_d;  // end-effector desired orientation

        Eigen::Matrix4d O_T_EE; // Transformation matrix referred to robot
        Eigen::Matrix4d T_spacenav;  //  Transformation matrix referred to spacenav
        Eigen::Matrix4d Tx, Ty, Tz;

        int spacenav_button_1 = 0;
        int spacenav_button_2 = 0;

        Eigen::Vector2d fingers_position;

        ros::Publisher pose_pub; // Panda Pose publisher
        ros::Publisher mf_pub; // Panda move fingers publisher
      
        // PUBLIC FUNCS
        void MotionControl(Eigen::Matrix4d& Xd); // Function to execute motion control
        void posePublisherCallback(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);
        Eigen::VectorXd robotPose(Eigen::Matrix4d& Xd);  // Function to get the current robot pose
        void moveFingersCallback(Eigen::Vector2d& position); // Function to open/close panda fingers
        visualization_msgs::Marker pointsMarker(std::string points_ns, int points_id, Eigen::Vector2d points_scale, Eigen::Vector3d points_color);
        visualization_msgs::Marker lineStripsMarker(std::string lines_ns, int lines_id, double lines_scale, Eigen::Vector3d lines_color);
        int inpolygon(const Eigen::MatrixXd &vertices, double x, double y); // Function to verify if the point (x,y) is inside the polygon
        Eigen::Matrix3d points2Rotation(Eigen::Vector3d& P1, Eigen::Vector3d& P2, Eigen::Vector3d& P3);

        /**
         * 
         * @param[in] pi - initial position
         * @param[in] pf - final position
         * @param[in] t - time
         * @param[in] ti - initial time
         * @param[in] tf - final time
         * @param[out] pd - desired position
         */ 
        Eigen::Vector3d polynomial3Trajectory(Eigen::Vector3d& pi, Eigen::Vector3d& pf, double ti, double tf, double t); 

        /**
         * 
         * @param[in] pi - initial position
         * @param[in] pf - final position
         * @param[in] t - time
         * @param[out] pd - desired position
         */ 
        Eigen::Vector3d lineTrajectory(Eigen::Vector3d& pi, Eigen::Vector3d& pf, double t);

        /**
         * @param[in] pi - initial position
         * @param[in] a - radio
         * @param[in] b - radio
         * @param[in] T - ellipse amplitude
         * @param[in] t - time 
         * @param[in] axis - 'XY', 'XZ' and 'YZ'
         * @param[out] pd - desired position
         */
        Eigen::Vector3d ellipseTrajectory(Eigen::Vector3d& pi, double a, double b, double T, double t, std::string axis);

        /**
         * @param[in] angle (rad)
         * @param[in] 0: rotation in 'X', 1: rotation in 'Y', 3: rotation in 'Z'
         * @param[out] Rd - desired rotation
         */
        Eigen::Matrix3d rotate(double angle, int axis);

    private:
        // PRIVATE VARIABLES
        ros::NodeHandle nh_; // private node NodeHandle
        ros::Subscriber spacenav_sub; // Spacenav Subscriber
        ros::Subscriber franka_state_sub; // Franka State Subscriber
        ros::Time time_lapse, last_time; // Time variables to change modes with spacenav

        int flag_mode = 0; // Flag to control robot in base frame or end-effector frame
        bool flag_motion = false; // Flag to determine if there is spacenav motion
        int panda_state_flag = 0;

        // PRIVATE FUNCS
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void pandaStateCallback(const geometry_msgs::PoseStampedConstPtr& msg);

};

} // namespace franka_simulation


#endif // SPACENAV_H
