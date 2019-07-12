// ============================================================================
// Name        : spacenav.h
// Author      : Hélio Ochoa
// Description : Permits the user control the robot end-effector position and
//               orientation and other funcionalities
// ============================================================================

#ifndef SPACENAV_H
#define SPACENAV_H

#include "ros/ros.h"
#include <sstream>
#include <ros/time.h>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <SFML/Window/Keyboard.hpp>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/MoveAction.h>
#include <math.h>

#define PI  3.14159265358979323846  /* pi */
#define CLEANWINDOW "\e[2J\e[H"

using namespace std;
using namespace Eigen;

namespace franka_spacenav {

  class Spacenav{
    public:
      Spacenav(ros::NodeHandle &nh);
      ~Spacenav();

      // VARIABLES
      Matrix<double, 6, 1> spacenav_motion; // Vector with spacenav motion
      Matrix4d FK; // O_T_EE -> Forward Kinematics

      Vector3d position_d;  // end-effector desired position
      Quaterniond orientation_d;  // end-effector desired orientation

      Matrix4d T_spacenav;  //  Transformation matrix referred to spacenav
      Matrix4d Tx, Ty, Tz;

      string link_name;

      double width = 0.0; // default gripper width
      double speed = 0.1; // default gripper speed
      double linear_y = 0.0;
      double angular_z = 0.0;

      int spacenav_button_1 = 0;
      int spacenav_button_2 = 0;

      ros::Publisher pose_pub; // Franka Pose publisher

      // FUNCS
      void MotionControl(Matrix4d &Xd); // Function to execute motion control
      void createSubscribersAndPublishers(void); // Function to robot publishers and subscribers
      void publisherCallback(geometry_msgs::PoseStamped &marker_pose, Vector3d &position, Quaterniond &quaternion);
      void quaternion2EulerAngles(Quaterniond &quaternion); // Function to convert quaternions in euler angles
      void gripper_move_client(double &width, double &speed); // Function to move gripper
      VectorXd polynomial3_trajectory(const VectorXd &pi, const VectorXd &pf, double ti, double tf, double t);
      VectorXd robot_pose(Matrix4d &O_T_EE);  // Function to get the current robot pose
      Vector3d polynomial3_traj3D(Vector3d& pi, Vector3d& pf, double ti, double tf, double t);
      Matrix3d rotate(double angle, int axis);  // rotate in any axis(x,y,z)

    private:
      // VARIABLES
      ros::NodeHandle nh_;  // private node NodeHandle
      ros::Subscriber spacenav_sub; // Spacenav Subscriber
      ros::Subscriber franka_state_sub; // Franka State Subscriber
      ros::Time time_lapse, last_time_mode, last_time_gripper; // Time variables to change modes from spacenav

      int flag_mode = 0;  // Flag to control spacenav motion from base or end effector
      int franka_state_flag = 0;
      bool flag_motion = false; // Flag to determine if there is spacenav motion


      // FUNCS
      void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);
      void franka_state_callback(const franka_msgs::FrankaState::ConstPtr &msg);


  };

}


#endif // SPACENAV_H
