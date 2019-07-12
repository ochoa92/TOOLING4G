// ============================================================================
// Name        : new_cart_imp_controller.h
// Author      : Hélio Ochoa
// Description : Orientation control based in a rotation error
// ============================================================================
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#define PI           3.14159265358979323846  /* pi */

namespace franka_spacenav {

class NewCartImpController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {

  public:
    NewCartImpController();
    ~NewCartImpController();

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;


  private:
    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    double filter_params_{0.005};
    double nullspace_stiffness_;
    double nullspace_damping_;
    const double delta_tau_max_{1.0};
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond Quaternion_d_;
    Eigen::Matrix3d R_d_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond Quaternion_d_target_;
    Eigen::Vector3d r_vector; // rotation vector
    Eigen::Matrix<double, 6, 1> velocity_d_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;

    int count; // file counter
    std::ofstream file;
    std::ifstream infile_gains;

    // Convert a rotation Matrix in rotation vector
    Eigen::Vector3d R2r(Eigen::Matrix3d& Rotation);

    // Get the RPY(ZYX) from a rotation Matrix
    Eigen::Vector3d R2RPY(Eigen::Matrix3d& Rotation);

    // Wrap between [-PI, PI]
    double wrapToPI(double& angle);

    // Wrap between [0, 2*PI]
    double wrapTo2PI(double& angle);

    // Equilibrium pose subscriber
    ros::Subscriber sub_equilibrium_pose_;
    void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);


};

}  // namespace franka_spacenav
