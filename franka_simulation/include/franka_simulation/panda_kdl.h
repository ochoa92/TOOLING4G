// ============================================================================
// Name        : panda_kdl.h
// Author      : Hélio Ochoa
// Description : A library that performs the calculations for the FK, DK and
//               Dynamics using the KDL library.
// ============================================================================

#ifndef PANDA_KDL_H
#define PANDA_KDL_H
#include <ros/ros.h>
#include <tf/tf.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <Eigen/Core>

#include <sensor_msgs/JointState.h>

using namespace KDL;

namespace P_KDL{

  class panda_kdl{
    public:
      panda_kdl(ros::NodeHandle &nh);
      ~panda_kdl();

      KDL::Chain kdl_chain;
      Eigen::Matrix<double, 7, 1> joint_position;
      Eigen::Matrix<double, 7, 1> joint_velocity;
      Eigen::Matrix<double, 7, 1> joint_effort;

      int num_joints; // number of joints

      bool FK(KDL::Frame &kdl_frame); // FK with current joint states
      bool FK(KDL::Frame &kdl_frame, Eigen::Matrix<double, 7, 1> q_values); // FK with given joint states

      bool jacobian(KDL::Jacobian &kdl_jacobian); // Jacobian with current joint states
      bool jacobian(KDL::Jacobian &kdl_jacobian, Eigen::Matrix<double, 7, 1> q_values);  // Jacobian with given joint states

      bool dynamic(KDL::JntSpaceInertiaMatrix &kdl_inertia, KDL::JntArray &kdl_coriolis, KDL::JntArray &kdl_gravity, Vector gravity); // Dynamic with current joint states
      bool dynamic(KDL::JntSpaceInertiaMatrix &kdl_inertia, KDL::JntArray &kdl_coriolis, KDL::JntArray &kdl_gravity, Eigen::Matrix<double, 7, 1> q_values, Eigen::Matrix<double, 7, 1> dq_values, Vector gravity); // Dynamic with given joint states

    private:
      ros::NodeHandle nh_;
      ros::Subscriber joint_states_sub;     // joint states subscriber
      int joint_state_flag = 0;
      std::string root_name, end_effector_name;
      KDL::Tree kdl_tree;

      void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

  };

}
#endif  // PANDA_KDL_H
