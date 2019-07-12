// =============================================================================
// Name        : cartesian_impedance_controller.h
// Author      : Hélio Ochoa
// Description : Cartesian impedance controller where the orientation control
//               is based on rotation error
// =============================================================================
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>

#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include "ros/ros.h"
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka/robot_state.h>
#include <franka/robot.h>

#include <geometry_msgs/PoseStamped.h>

#define PI  3.14159265358979323846  /* pi */


namespace franka_spacenav {

class CartesianImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:

  CartesianImpedanceController();
  ~CartesianImpedanceController();

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;


 private:
  // Saturation ----------------------------------------------------------------
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                                 const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  // handles -------------------------------------------------------------------
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // control variables ---------------------------------------------------------
  double filter_params_{0.01};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 7, 7> nullspace_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_integral_;
  Eigen::Vector3d position_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Matrix3d R_d_;
  Eigen::Quaterniond Quaternion_d_;
  Eigen::Quaterniond Quaternion_d_target_;
  Eigen::Matrix<double, 6, 1> velocity_d_;
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> velocity_error;
  Eigen::Matrix<double, 6, 1> integral_error;
  Eigen::Matrix<double, 6, 1> last_integral_error;
  double Kpx, Kpy, Kpz, Kox, Koy, Koz, Dpx, Dpy, Dpz, Dox, Doy, Doz, Kp_nullspace, Ipx, Ipy, Ipz, Iox, Ioy, Ioz;
  Eigen::Matrix3d Kp_d_, Dp_d_; // position stiffness and damping in desired frame
  Eigen::Matrix3d Ko_d_, Do_d_; // orientation stiffness and damping in desired frame
  Eigen::Matrix3d Ip_d_, Io_d_; // position and orientation integral in desired frame
  Eigen::Matrix<double, 7, 1> maxJointLimits;
  Eigen::Matrix<double, 7, 1> minJointLimits;
  Eigen::Matrix<double, 7, 1> gradient;  // gradient mechanical joint limit

  // file with gains and file for tracking -------------------------------------
  int count; // file counter
  std::ofstream file_tracking;
  std::ifstream file_gains;

  // important functions -------------------------------------------------------
  Eigen::Vector3d R2r(Eigen::Matrix3d& Rotation);   // Convert a rotation Matrix in rotation vector
  Eigen::Vector3d R2EulerAngles(Eigen::Matrix3d& Rotation); // Convert rotation matrix to euler angles(XYZ->YPR)
  double wrapToPI(double& angle); // Wrap between [-PI, PI]
  double wrapTo2PI(double& angle);  // Wrap between [0, 2*PI]

  // Equilibrium pose subscriber -----------------------------------------------
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // posture optimization funcs ------------------------------------------------
  double derivative_computation( const double q_i, const double maxJointLimit_i, const double minJointLimit_i);
  template<int N> // number of joints or DOF
  void gradient_mechanical_joint_limit( Eigen::Matrix<double, N, 1>& gradient_mechanical_joint_limit_out, const Eigen::Matrix<double, N, 1> q, const Eigen::Matrix<double, N, 1> maxJointLimits, const Eigen::Matrix<double, N, 1> minJointLimits );

};

}  // namespace franka_spacenav
