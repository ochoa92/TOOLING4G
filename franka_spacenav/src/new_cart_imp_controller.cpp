#include <franka_spacenav/new_cart_imp_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot.h>

#include "pseudo_inversion.h"


namespace franka_spacenav {


NewCartImpController::NewCartImpController(){
  std::cout << "Open the file to write!" << std::endl;
  std::string path;
  path = "/home/panda/kst/franka/new_cart_imp_controller";
  file.open(path,std::ofstream::out);
  file << " t; p_x; p_xd; p_y; p_yd; p_z; p_zd; Yaw(X); Yaw_d(Xd); Pitch(Y); Pitch_d(Yd); Roll(Z); Roll(Zd); v_x; v_xd; v_y; v_yd; v_z; v_zd; w_x; w_xd; w_y; w_yd; w_z; w_zd; r_x; r_xd; r_y; r_yd; r_z; r_zd; Fx; Fy; Fz\n";
  file << " s; m; m; m; m; m; m; rad; rad; rad; rad; rad; rad; m/s; m/s; m/s; m/s; m/s; m/s; rad/s; rad/s; rad/s; rad/s; rad/s; rad/s; rad; rad; rad; rad; rad; rad; N; N; N\n";
}

NewCartImpController::~NewCartImpController(){
  std::cout << "File closed!" << std::endl << std::endl;
  file.close();
}


bool NewCartImpController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle){

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("NewCartImpController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("NewCartImpController: Invalid or no joint_names parameters provided, aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("NewCartImpController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("NewCartImpController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("NewCartImpController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("NewCartImpController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("NewCartImpController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("NewCartImpController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  sub_equilibrium_pose_ = node_handle.subscribe("/panda_equilibrium_pose", 20, &NewCartImpController::equilibriumPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());


  // ---------------------------------------------------------------------------
  // init values
  // ---------------------------------------------------------------------------
  position_d_.setZero();
  Quaternion_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  R_d_.setZero();

  position_d_target_.setZero();
  Quaternion_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  nullspace_stiffness_ = 0.0;
  nullspace_damping_ = 0.0;

  velocity_d_.setZero();
  r_vector.setZero();

  count = 0;
  // ---------------------------------------------------------------------------

  return true;
}



void NewCartImpController::starting(const ros::Time& /*time*/) {

  // compute initial velocity with jacobian and set x_attractor to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  Quaternion_d_ = Eigen::Quaterniond(initial_transform.linear());
  R_d_ = initial_transform.rotation();
  position_d_target_ = initial_transform.translation();
  Quaternion_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;

  // ---------------------------------------------------------------------------
  // set the controller gains
  // ---------------------------------------------------------------------------
  std::string path_gains;
  double k1, k2, k3, k4, k5, k6, k7;
  path_gains = "/home/panda/catkin_ws/src/TOOLING4G/franka_spacenav/gains/new_cart_imp_controller_gains";
  infile_gains.open(path_gains);
  if(infile_gains.is_open()){
    infile_gains >> k1 >> k2 >> k3 >> k4 >> k5 >> k6 >> k7;
  }
  else{
    std::cout << "Error open the file!" << std::endl;
  }
  infile_gains.close();

  double Kpx, Kpy, Kpz, Kox, Koy, Koz, Dpx, Dpy, Dpz, Dox, Doy, Doz;
  // PD GAINS -> NO Inertia Shaping
  Kpx = k1;
  Kpy = k2;
  Kpz = k3;
  Kox = k4;
  Koy = k5;
  Koz = k6;

  // Damping ratio = 1
  Dpx = 2.0 * sqrt(Kpx);
  Dpy = 2.0 * sqrt(Kpy);
  Dpz = 2.0 * sqrt(Kpz);
  Dox = 2.0 * sqrt(Kox);
  Doy = 2.0 * sqrt(Koy);
  Doz = 2.0 * sqrt(Koz);

  // Spring(K) and Damper(D) matrices
  cartesian_stiffness_ << Kpx,   0,   0,   0,   0,   0,
                            0, Kpy,   0,   0,   0,   0,
                            0,   0, Kpz,   0,   0,   0,
                            0,   0,   0, Kox,   0,   0,
                            0,   0,   0,   0, Koy,   0,
                            0,   0,   0,   0,   0, Koz;

  cartesian_damping_ << Dpx,   0,   0,   0,   0,   0,
                          0, Dpy,   0,   0,   0,   0,
                          0,   0, Dpz,   0,   0,   0,
                          0,   0,   0, Dox,   0,   0,
                          0,   0,   0,   0, Doy,   0,
                          0,   0,   0,   0,   0, Doz;

  // nullspace Gains
  nullspace_stiffness_ = k7;
  nullspace_damping_ = 2.0 * sqrt(nullspace_stiffness_);

  // ---------------------------------------------------------------------------

}


void NewCartImpController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array = model_handle_->getMass();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data()); // NOLINT (readability-identifier-naming)
  Eigen::Map<Eigen::Matrix<double, 6, 1>> ext_EE_F(robot_state.K_F_ext_hat_K.data()); // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix3d R(transform.rotation());

  // ---------------------------------------------------------------------------
  // compute error to desired pose
  // ---------------------------------------------------------------------------
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position_d_ - position;

  // orientation error
  Eigen::Matrix3d cRcd(R.inverse() * R_d_); // described in current end-effector frame
  Eigen::Matrix3d Rcd(R * cRcd * R.inverse()); // described in the base frame
  error.tail(3) << R2r(Rcd);


  // velocity error
  Eigen::Matrix<double, 6, 1> velocity_error;
  Eigen::Matrix<double, 6, 1> velocity(jacobian * dq);
  velocity_error << velocity_d_ - velocity;

  // ---------------------------------------------------------------------------
  // compute control
  // ---------------------------------------------------------------------------
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * ( cartesian_damping_ * velocity_error + cartesian_stiffness_ * error );


  // compute the inertia matrix of the task space
  // Eigen::Matrix<double, 6, 6> lambda_inv(jacobian * mass.inverse() * jacobian.transpose());
  // Eigen::Matrix<double, 6, 6> lambda(lambda_inv.inverse());
  // tau_task << jacobian.transpose() * lambda * ( cartesian_damping_ * velocity_error + cartesian_stiffness_ * error );


  // pseudoinverse for nullspace handling
  // kinematic pseudoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) * (nullspace_stiffness_ * (q_d_nullspace_ - q) - nullspace_damping_ * dq);

  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // std::cout << "\n" << tau_d << std::endl;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // ---------------------------------------------------------------------------
  // update parameters
  // ---------------------------------------------------------------------------
  // target by filtering
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  Quaternion_d_.coeffs() = filter_params_ * Quaternion_d_target_.coeffs() + (1.0 - filter_params_) * Quaternion_d_.coeffs();
  R_d_ = Quaternion_d_.toRotationMatrix();

  // ---------------------------------------------------------------------------
  // Write to file
  // ---------------------------------------------------------------------------
  Eigen::Vector3d euler_angles(R.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)
  Eigen::Vector3d euler_angles_d_(R_d_.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)
  // Eigen::Vector3d euler_angles(R2RPY(R)); // XYZ
  // Eigen::Vector3d euler_angles_d_(R2RPY(R_d_)); // XYZ
  Eigen::Vector3d r(R2r(R)); // r = angle*axis
  Eigen::Vector3d r_d_(R2r(R_d_)); // r_d = angle*axis
  count++;
  double TIME = count/1000.0;
  file << " " << TIME << "; "
              << position[0] << "; " << position_d_[0] << "; "
              << position[1] << "; " << position_d_[1] << "; "
              << position[2] << "; " << position_d_[2] << "; "
              << wrapToPI(euler_angles[0]) << "; " << wrapToPI(euler_angles_d_[0]) << "; "
              << wrapToPI(euler_angles[1]) << "; " << wrapToPI(euler_angles_d_[1]) << "; "
              << wrapToPI(euler_angles[2]) << "; " << wrapToPI(euler_angles_d_[2]) << "; "
              << velocity[0] << "; " << velocity_d_[0] << "; "
              << velocity[1] << "; " << velocity_d_[1] << "; "
              << velocity[2] << "; " << velocity_d_[2] << "; "
              << velocity[3] << "; " << velocity_d_[3] << "; "
              << velocity[4] << "; " << velocity_d_[4] << "; "
              << velocity[5] << "; " << velocity_d_[5] << "; "
              << r[0] << "; " << r_d_[0] << "; "
              << r[1] << "; " << r_d_[1] << "; "
              << r[2] << "; " << r_d_[2] << "; "
              << ext_EE_F[0] << "; "
              << ext_EE_F[1] << "; "
              << ext_EE_F[2] << "\n";

}


Eigen::Vector3d NewCartImpController::R2RPY(Eigen::Matrix3d& Rotation){
  double Roll, Pitch , Yaw; // RPY -> ZYX
  Eigen::Vector3d XYZ;

  Roll = atan2(Rotation(1,0), Rotation(0,0));
  Pitch = atan2(-Rotation(2,0), sqrt(Rotation(2,1)*Rotation(2,1) + Rotation(2,2)*Rotation(2,2)));
  Yaw = atan2(Rotation(2,1), Rotation(2,2));

  if(Pitch > PI/2 && Pitch < 3*PI/2 ){
    Roll = atan2(-Rotation(1,0), -Rotation(0,0));
    Pitch = atan2(-Rotation(2,0), -sqrt(Rotation(2,1)*Rotation(2,1) + Rotation(2,2)*Rotation(2,2)));
    Yaw = atan2(-Rotation(2,1), -Rotation(2,2));
  }
  XYZ << Yaw,
         Pitch,
         Roll;

  return XYZ;
}


Eigen::Vector3d NewCartImpController::R2r(Eigen::Matrix3d &Rotation){
  Eigen::Vector3d aux;
  aux << Rotation(2,1) - Rotation(1,2),
         Rotation(0,2) - Rotation(2,0),
         Rotation(1,0) - Rotation(0,1);
  r_vector = 0.5 * aux;

  return r_vector;
}


Eigen::Matrix<double, 7, 1> NewCartImpController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}


void NewCartImpController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z;
  Eigen::Quaterniond last_Quaternion_d_target(Quaternion_d_target_);
  Quaternion_d_target_.coeffs() << msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w;
  if (last_Quaternion_d_target.coeffs().dot(Quaternion_d_target_.coeffs()) < 0.0) {
    Quaternion_d_target_.coeffs() << -Quaternion_d_target_.coeffs();
  }
}


double NewCartImpController::wrapToPI(double& angle){
  double new_angle = atan2(sin(angle), cos(angle));
  return new_angle;
}


double NewCartImpController::wrapTo2PI(double& angle){
  double new_angle = asin(sin(angle));
  if(cos(angle) < 0){
    new_angle = M_PI-new_angle;
  }
  else if(new_angle < 0){
    new_angle += 2*M_PI;
  }
  return new_angle;
}


} // namespace franka_spacenav

PLUGINLIB_EXPORT_CLASS(franka_spacenav::NewCartImpController,
                       controller_interface::ControllerBase)
