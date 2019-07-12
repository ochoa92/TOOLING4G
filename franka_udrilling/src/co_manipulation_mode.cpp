#include <franka_udrilling/co_manipulation_mode.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot.h>

// STATE MACHINE --------------------------------------------------------------
#define WAIT4STATION 0
#define STATION 1
#define WAIT4ORIENTATION 2
#define MOULD_ORIENTATION 3
#define WAIT4POINTS 4
#define MOULD_POINTS 5
// -----------------------------------------------------------------------------


namespace franka_udrilling {


CoManipulationMode::CoManipulationMode(){
  std::cout << "ROBOT READY TO SAVE DATA!" << std::endl;
  // mould points file opened
  mould_points_file.open("/home/helio/catkin_ws/src/franka_ros/franka_udrilling/co_manipulation_data/mould_points", std::ofstream::out);
}


CoManipulationMode::~CoManipulationMode(){
  std::cout << "ALL FILES CLOSED!" << std::endl << std::endl;
  station_file.close();  // station file close
  mould_orientation_file.close();  // mould orientation file close
  mould_points_file.close(); // mould points file close
}


bool CoManipulationMode::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle){

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CoManipulationMode: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("CoManipulationMode: Invalid or no joint_names parameters provided, "
              "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("CoManipulationMode: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  }
  catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("CoManipulationMode: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("CoManipulationMode: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  }
  catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("CoManipulationMode: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("CoManipulationMode: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("CoManipulationMode: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  spacenav_sub = node_handle.subscribe("/spacenav/joy", 20, &CoManipulationMode::joyCallback, this);

  marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 20);
  points.header.frame_id = "/panda_link0";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.id = 0;
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::POINTS;
  points.pose.orientation.w = 1.0;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.01;
  points.scale.y = 0.01;
  points.scale.z = 0.01;

  // Set the color -- be sure to set alpha to something non-zero!
  points.color.a = 1.0;
  points.color.r = 1.0;
  points.color.g = 0.0;
  points.color.b = 0.0;

  return true;
}


void CoManipulationMode::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data()); // NOLINT (readability-identifier-naming)
  Eigen::Map<Eigen::Matrix<double, 6, 1>> EE_force(robot_state.K_F_ext_hat_K.data()); // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame
  Eigen::Map<Eigen::Matrix<double, 6, 1>> O_force(robot_state.O_F_ext_hat_K.data()); // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // allocate variables
  Eigen::VectorXd tau_task(7), tau_d(7);
  tau_task << 0, 0, 0, 0, 0, 0, 0;

  // Desired torque
  tau_d << tau_task;
  //std::cout << "\n" << tau_d << std::endl;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // ---------------------------------------------------------------------------
  // flag_mode
  // ---------------------------------------------------------------------------
  ros::Duration d(0.5);
  time_lapse = ros::Time::now();

  switch (flag_mode) {  ////////////////////////////////////////////////////////

    // -------------------------------------------------------------------------
    case WAIT4STATION:
      if(flag_print == 0){
        std::cout << CLEANWINDOW << "WOULD YOU LIKE TO SAVE A NEW STATION?" << std::endl;
        std::cout << "IF <YES> PLEASE PRESS SPACENAV BUTTON <1>!" << std::endl;
        std::cout << "IF <NO> PLEASE PRESS SPACENAV BUTTON <2>!" << std::endl;
        flag_print = 1;
      }

      if(spacenav_button_1 == 1 && time_lapse - last_time > d){
        last_time = time_lapse;
        flag_mode = STATION;
        // station file opened
        station_file.open("/home/helio/catkin_ws/src/franka_ros/franka_udrilling/co_manipulation_data/station", std::ofstream::out);
      }

      if(spacenav_button_2 == 1 && time_lapse - last_time > d){
        last_time = time_lapse;
        flag_mode = WAIT4ORIENTATION;
        flag_print = 2;
      }

      break;

    // -------------------------------------------------------------------------
    case STATION:
      if(flag_print == 1){
        std::cout << CLEANWINDOW << "NEW STATION SAVED! PLEASE PRESS SPACENAV BUTTON <1> TO CONTINUE..." << std::endl;
        station_file << position[0] << " " << position[1] << " " << position[2] << "\n";
        flag_print = 2;
      }

      if(spacenav_button_1 == 1 && time_lapse - last_time > d){
        last_time = time_lapse;
        flag_mode = WAIT4ORIENTATION;
      }

      break;

    // -------------------------------------------------------------------------
    case WAIT4ORIENTATION:
      if(flag_print == 2){
        std::cout << CLEANWINDOW << "WOULD YOU LIKE TO SAVE A NEW MOULD ORIENTATION?" << std::endl;
        std::cout << "IF <YES> PLEASE PRESS SPACENAV BUTTON <1>!" << std::endl;
        std::cout << "IF <NO> PLEASE PRESS SPACENAV BUTTON <2>!" << std::endl;
        flag_print = 3;
      }

      if(spacenav_button_1 == 1 && time_lapse - last_time > d){
        last_time = time_lapse;
        flag_mode = MOULD_ORIENTATION;
        // mould orientation file opened
        mould_orientation_file.open("/home/helio/catkin_ws/src/franka_ros/franka_udrilling/co_manipulation_data/mould_orientation", std::ofstream::out);
      }

      if(spacenav_button_2 == 1 && time_lapse - last_time > d){
        last_time = time_lapse;
        flag_mode = WAIT4POINTS;
        flag_print = 4;
      }

      break;

    // -------------------------------------------------------------------------
    case MOULD_ORIENTATION:
      if(flag_print == 3){
        std::cout << CLEANWINDOW << "NEW MOULD ORIENTATION SAVED! PLEASE PRESS SPACENAV BUTTON <1> TO CONTINUE..." << std::endl;
        mould_orientation_file << orientation.vec()[0] << " " << orientation.vec()[1] << " " << orientation.vec()[2] << " " << orientation.w() << "\n";
        flag_print = 4;
      }

      if(spacenav_button_1 == 1 && time_lapse - last_time > d){
        last_time = time_lapse;
        flag_mode = WAIT4POINTS;
      }

      break;

    // -------------------------------------------------------------------------
    case WAIT4POINTS:
      if(flag_print == 4){
        std::cout << CLEANWINDOW << "PLEASE PRESS SPACENAV BUTTON <1> TO SAVE A MOULD POINT!" << std::endl;
        flag_print = 5;
      }

      if(spacenav_button_1 == 1 && time_lapse - last_time > d){
        last_time = time_lapse;
        flag_mode = MOULD_POINTS;
      }

      break;

    // -------------------------------------------------------------------------
    case MOULD_POINTS:
      if(flag_print == 5){
        mould_points_file << "" << position[0] << " " << position[1] << " " << position[2] << "\n";
        p.x = position[0];
        p.y = position[1];
        p.z = position[2];
        points.points.push_back(p);
        std::cout << CLEANWINDOW << "POINT SAVED! PLEASE PRESS SPACENAV BUTTON <1> TO CONTINUE..." << std::endl;
        flag_print = 4;
      }

      if(spacenav_button_1 == 1 && time_lapse - last_time > d){
        last_time = time_lapse;
        flag_mode = WAIT4POINTS;
      }

      break;

  } ////////////////////////////////////////////////////////////////////////////

  marker_pub.publish(points); // Draw the points in rviz

}


Eigen::Matrix<double, 7, 1> CoManipulationMode::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}


void CoManipulationMode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  spacenav_button_1 = msg->buttons[0];
  spacenav_button_2 = msg->buttons[1];
}


} // namespace franka_udrilling

PLUGINLIB_EXPORT_CLASS(franka_udrilling::CoManipulationMode, controller_interface::ControllerBase)
