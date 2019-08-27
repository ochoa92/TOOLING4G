// =============================================================================
// Name        : udrilling_controller.h
// Author      : Hélio Ochoa
// Description : A controller for micro-drilling tasks based on a cartesian 
//               impedance controller with posture optimization. Compliance 
//               parameters and the equilibrium pose can be modified online.
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
#include <franka/robot_state.h>
#include <franka/robot.h>

#include "ros/ros.h"
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>

#include <Eigen/Dense>

#include <franka_udrilling/compliance_paramConfig.h>


#define PI  3.14159265358979323846  /* pi */


namespace franka_udrilling {

class uDrillingController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                                                  hardware_interface::EffortJointInterface,
                                                                                  franka_hw::FrankaStateInterface> {
    public:
        // constructor and destructor
        uDrillingController();
        ~uDrillingController();

        // inherited virtual functions
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;

    private:
        // saturation 
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                                       const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

        // handles 
         std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;  

        // controller variables 
        double filter_params_{0.01};
        const double delta_tau_max_{1.0};
        Eigen::Vector3d position_d_;
        Eigen::Vector3d position_d_target_;
        Eigen::Matrix3d R_d_;
        Eigen::Quaterniond orientation_d_;
        Eigen::Quaterniond orientation_d_target_;
        Eigen::Matrix<double, 6, 1> velocity_d_;
        Eigen::Matrix<double, 6, 1> error;
        Eigen::Matrix<double, 6, 1> velocity_error;
        Eigen::Matrix<double, 6, 1> integral_error;
        Eigen::Matrix<double, 6, 1> last_integral_error;
        Eigen::Matrix<double, 7, 1> maxJointLimits;
        Eigen::Matrix<double, 7, 1> minJointLimits;
        Eigen::Matrix<double, 7, 1> gradient; // gradient mechanical joint limit   
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_;
        Eigen::Matrix<double, 6, 6> cartesian_integral_;
        Eigen::Matrix3d Kp_d_, Dp_d_; // position stiffness and damping in desired frame
        Eigen::Matrix3d Kp_d_target_, Dp_d_target_;
        Eigen::Matrix3d Ko_d_, Do_d_; // orientation stiffness and damping in desired frame
        Eigen::Matrix3d Ko_d_target_, Do_d_target_;
        Eigen::Matrix3d Ip_d_, Io_d_; // position and orientation integral in desired frame
        Eigen::Matrix3d Ip_d_target_, Io_d_target_;
        Eigen::Matrix<double, 7, 7> nullspace_stiffness_;
        Eigen::Matrix<double, 7, 7> nullspace_stiffness_target_;

        // tracking file
        int count; // file counter
        std::ofstream tracking_file;

        // important funcs 
        Eigen::Vector3d R2r(Eigen::Matrix3d& Rotation); // Convert a rotation Matrix in rotation vector
        double wrapToPI(double& angle); // Wrap between [-PI, PI]
        double wrapTo2PI(double& angle); // Wrap between [0, 2*PI]

        // Equilibrium pose subscriber 
        ros::Subscriber sub_equilibrium_pose_;
        void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

        // Dynamic reconfigure
        std::unique_ptr<dynamic_reconfigure::Server<franka_udrilling::compliance_paramConfig>> dynamic_server_compliance_param_;
        ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
        void complianceParamCallback(franka_udrilling::compliance_paramConfig& config, uint32_t level);

        // posture optimization funcs 
        double derivative_computation(const double q_i, const double maxJointLimit_i, const double minJointLimit_i);
        template<int N> // number of joints or DOF
        void gradient_mechanical_joint_limit(Eigen::Matrix<double, N, 1>& gradient_mechanical_joint_limit_out, 
                                             const Eigen::Matrix<double, N, 1> q, 
                                             const Eigen::Matrix<double, N, 1> maxJointLimits, 
                                             const Eigen::Matrix<double, N, 1> minJointLimits );                                                                               

};

}  // namespace franka_udrilling