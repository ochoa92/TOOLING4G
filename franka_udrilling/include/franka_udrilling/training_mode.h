// =============================================================================
// Name        : training_mode.h
// Author      : Hélio Ochoa
// Description : Switches to gravity compensation and allows the user to 
//               manipulate the robot               
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

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <franka/robot_state.h>
#include <franka/robot.h>

#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>

#define CLEANWINDOW "\e[2J\e[H"


namespace franka_udrilling {

class TrainingMode : public controller_interface::MultiInterfaceController< franka_hw::FrankaModelInterface,
                                                                            hardware_interface::EffortJointInterface,
                                                                            franka_hw::FrankaStateInterface > {

  public:
    TrainingMode();
    ~TrainingMode();

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;

  private:
    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                                   const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    const double delta_tau_max_{1.0};

    int count; // file counter
    std::ofstream file;

    // Pose publisher
    ros::Publisher pose_pub;
    geometry_msgs::PoseStamped robot_pose;
    void posePublisherCallback(ros::Publisher& pose_pub, geometry_msgs::PoseStamped& robot_pose, Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

};

}  // namespace franka_udrilling
