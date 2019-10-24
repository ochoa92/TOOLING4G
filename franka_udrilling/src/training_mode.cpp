#include <franka_udrilling/training_mode.h>

namespace franka_udrilling {


TrainingMode::TrainingMode(){
  std::cout << "Open the files to write!" << std::endl;
  std::string path;
  path = "/home/panda/kst/udrilling/user_pattern";
  file.open(path, std::ofstream::out);
  file << " t p_x p_y p_z Q_x Q_y Q_z Q_w Fx Fy Fz pEE_x pEE_y pEE_z\n";
  file << " s m m m Qunit Qunit Qunit Qunit N N N m m m\n";
}


TrainingMode::~TrainingMode(){
  std::cout << "Files closed!" << std::endl << std::endl;
  file.close();
}


bool TrainingMode::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle){

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("TrainingMode: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("TrainingMode: Invalid or no joint_names parameters provided, "
              "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("TrainingMode: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  }
  catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("TrainingMode: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("TrainingMode: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  }
  catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("TrainingMode: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("TrainingMode: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("TrainingMode: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Create publisher
  pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/robot_poseEE", 20);

  count = 0;

  return true;
}


void TrainingMode::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data()); // NOLINT (readability-identifier-naming)
  Eigen::Map<Eigen::Matrix<double, 6, 1>> EE_force(robot_state.K_F_ext_hat_K.data()); // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())); // Measured end effector pose in base frame.
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Matrix3d R(transform.rotation());

  Eigen::Vector3d positionEE(R*position); // Measured end effector pose in end effector frame.


  posePublisherCallback(pose_pub, robot_pose, position, orientation);

  // allocate variables
  Eigen::VectorXd tau_task(7), tau_d(7);
  tau_task << 0, 0, 0, 0, 0, 0, 0;

  // Desired torque
  tau_d << tau_task;
  // std::cout << "\n" << tau_d << std::endl;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // ---------------------------------------------------------------------------
  // SAVE DATA IN FILE
  // ---------------------------------------------------------------------------
  count++;
  double t = count/1000.0;
  file << " " << t << " "
       << position[0] << " "
       << position[1] << " "
       << position[2] << " "
       << orientation.vec()[0] << " "
       << orientation.vec()[1] << " "
       << orientation.vec()[2] << " "
       << orientation.w() << " "
       << EE_force[0] << " "
       << EE_force[1] << " "
       << EE_force[2] << " "
       << positionEE[0] << " "
       << positionEE[1] << " "
       << positionEE[2] << "\n";

}


Eigen::Matrix<double, 7, 1> TrainingMode::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}


void TrainingMode::posePublisherCallback(ros::Publisher& pose_pub, geometry_msgs::PoseStamped& robot_pose, Eigen::Vector3d& position, Eigen::Quaterniond& orientation){

  robot_pose.pose.position.x = position[0];
  robot_pose.pose.position.y = position[1];
  robot_pose.pose.position.z = position[2];

  robot_pose.pose.orientation.x = orientation.vec()[0];
  robot_pose.pose.orientation.y = orientation.vec()[1];
  robot_pose.pose.orientation.z = orientation.vec()[2];
  robot_pose.pose.orientation.w = orientation.w();

  // run pose publisher
  robot_pose.header.frame_id = "/panda_link0";
  robot_pose.header.stamp = ros::Time::now();
  pose_pub.publish(robot_pose);

}


} // namespace franka_udrilling

PLUGINLIB_EXPORT_CLASS(franka_udrilling::TrainingMode,
                       controller_interface::ControllerBase)
