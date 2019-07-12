// =============================================================================
// Name        : co_manipulation_mode.h
// Author      : Hélio Ochoa
// Description : The robot switches to gravity compensation and allows the user
//               to choose N planes and train/teach the robot.
// =============================================================================
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
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#define CLEANWINDOW "\e[2J\e[H"

namespace franka_polishing {

  class CoManipulationMode : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                                                   hardware_interface::EffortJointInterface,
                                                                                   franka_hw::FrankaStateInterface> {

    public:
      CoManipulationMode();
      ~CoManipulationMode();

      bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
      void update(const ros::Time&, const ros::Duration& period) override;

    private:

      // Saturation ------------------------------------------------------------
      Eigen::Matrix<double, 7, 1> saturateTorqueRate(
          const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
          const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

      const double delta_tau_max_{1.0};

      // handles ---------------------------------------------------------------
      std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
      std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
      std::vector<hardware_interface::JointHandle> joint_handles_;


      // co-manipulation variables ---------------------------------------------
      int t = 0; // file counter
      double TIME = 0;
      std::ofstream file;

      int t_pattern = 0;
      double TIME_PATTERN = 0;
      std::ofstream pattern;

      std::ofstream plane;

      int flag_mode = 0;
      int flag_print = 0;

      // spacenav_node variables and funcs -------------------------------------
      ros::Subscriber spacenav_sub;
      ros::Time time_lapse, last_time; // Time variables to change modes with spacenav
      int spacenav_button_1 = 0;
      int spacenav_button_2 = 0;
      void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

      // visualization markers variables ---------------------------------------
      ros::Publisher marker_pub;
      visualization_msgs::Marker points;
      geometry_msgs::Point p;  // points


  };

}  // namespace franka_polishing
