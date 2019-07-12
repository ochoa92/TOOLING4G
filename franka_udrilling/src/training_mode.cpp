#include <franka_udrilling/training_mode.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot.h>


namespace franka_udrilling {


TrainingMode::TrainingMode(){
  std::cout << "Open the files to write!" << std::endl;
  std::string path;
  path = "/home/helio/kst/udrilling/user_pattern";
  file.open(path, std::ofstream::out);
  file << " t p_x p_y p_z Qx Qy Qz Qw Fx Fy Fz\n";
  file << " s m m m Qunit Qunit Qunit Qunit N N N\n";

  // points
  std::string path_points;
  path_points = "/home/helio/kst/udrilling/mould/points";
  points.open(path_points, std::ofstream::out);

  // desired orientation
  std::string path_desired_o;
  path_desired_o = "/home/helio/kst/udrilling/mould/desired_o";
  desired_o.open(path_desired_o, std::ofstream::out);

}
TrainingMode::~TrainingMode(){
  std::cout << "Files closed!" << std::endl << std::endl;
  file.close();

  // points
  points.close();

  // desired orientation
  desired_o.close();
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

  spacenav_sub = node_handle.subscribe("/spacenav/joy", 20, &TrainingMode::joyCallback, this);

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

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  orientation.normalize();

  ros::Duration d(0.5);
  time_lapse = ros::Time::now();
  if (spacenav_button_1 == 1 && time_lapse - last_time > d) {
    last_time = time_lapse;
    if (flag_mode == 0)
      flag_mode = 1;
    else if (flag_mode == 1)
      flag_mode = 2;
    else if (flag_mode == 2)
      flag_mode = 3;
    else if (flag_mode == 3)
      flag_mode = 2;
  }

  // compute control
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

  switch (flag_mode) {
    case 0:
      if(flag_print == 0){
        std::cout << CLEANWINDOW << "PRESS SPACENAV BUTTON 1 TO SAVE THE DESIRED ORIENTATION..." << std::endl;
        flag_print = 1;
      }
      break;

    case 1:
      if(flag_print == 1){
        desired_o << " " << orientation.vec()[0] << " "
                         << orientation.vec()[1] << " "
                         << orientation.vec()[2] << " "
                         << orientation.w() << "\n";
        std::cout << CLEANWINDOW << "ORIENTATION SAVED! AGAIN TO SAVE THE FIRST POINT OF THE MOULD..." << std::endl;
        flag_print = 2;
      }
      break;

    case 2:
      if(flag_print == 2){
        points << " " << position[0] << " "
                      << position[1] << " "
                      << position[2] << "\n";
        std::cout << CLEANWINDOW << "POINT SAVED. CONTINUE PRESSING TO SAVE MORE POINTS..." << std::endl;
        flag_print = 3;
      }
      break;

    case 3:
      if(flag_print == 3){
        points << " " << position[0] << " "
                      << position[1] << " "
                      << position[2] << "\n";
        std::cout << CLEANWINDOW << "CONTINUE PRESSING TO SAVE MORE POINTS..." << std::endl;
        flag_print = 2;
      }
      break;
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
       << EE_force[2] << "\n";

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


void TrainingMode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  spacenav_button_1 = msg->buttons[0];
  spacenav_button_2 = msg->buttons[1];
}


} // namespace franka_udrilling

PLUGINLIB_EXPORT_CLASS(franka_udrilling::TrainingMode,
                       controller_interface::ControllerBase)
