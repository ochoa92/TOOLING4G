#include <franka_polishing/cartesian_impedance_controller.h>

namespace franka_polishing {


CartesianImpedanceController::CartesianImpedanceController(){
  std::cout << "Open the file to write!" << std::endl;
  std::string tracking_path;
  tracking_path = "/home/panda/kst/polishing/cartesian_impedance_controller";
  file_tracking.open(tracking_path, std::ofstream::out);
  file_tracking << " t p_x p_xd p_y p_yd p_z p_zd Yaw(X) Yaw_d(Xd) Pitch(Y) Pitch_d(Yd) Roll(Z) Roll_d(Zd) Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O e_px e_py e_pz e_ox e_oy e_oz pEE_x pEE_xd pEE_y pEE_yd pEE_z pEE_zd i_px i_py i_pz i_ox i_oy i_oz\n";
  file_tracking << " s m m m m m m rad rad rad rad rad rad N N N N N N m m m rad rad rad m m m m m m m m m rad rad rad\n";
}

CartesianImpedanceController::~CartesianImpedanceController(){
  std::cout << "File closed!" << std::endl << std::endl;
  file_tracking.close();
}


bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle){

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("CartesianImpedanceController: Invalid or no joint_names parameters provided, aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("CartesianImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  sub_equilibrium_pose_ = node_handle.subscribe("/panda_equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());


  // ---------------------------------------------------------------------------
  // init values
  // ---------------------------------------------------------------------------
  position_d_.setZero();
  R_d_.setZero();
  Quaternion_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  velocity_d_.setZero();

  position_d_target_.setZero();
  Quaternion_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  cartesian_integral_.setZero();
  nullspace_stiffness_.setZero();

  error.setZero();
  velocity_error.setZero();
  integral_error.setZero();
  last_integral_error.setZero();

  Kp_d_.setZero();
  Dp_d_.setZero();

  Ko_d_.setZero();
  Do_d_.setZero();

  Ip_d_.setZero();
  Io_d_.setZero();

  maxJointLimits << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  minJointLimits << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  gradient.setZero();

  count = 0;
  // ---------------------------------------------------------------------------

  return true;
}



void CartesianImpedanceController::starting(const ros::Time& /*time*/) {

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
  R_d_ = initial_transform.rotation();
  Quaternion_d_ = Eigen::Quaterniond(initial_transform.linear());

  position_d_target_ = initial_transform.translation();
  Quaternion_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // ---------------------------------------------------------------------------
  // Get the controller gains from a file
  // ---------------------------------------------------------------------------
  std::string path_gains;
  path_gains = "/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/controller_gains/compliance_param";
  file_gains.open(path_gains);
  std::string line;
  getline(file_gains, line);  // first line (ignore)
  if(file_gains.is_open()){
    file_gains >> Kpx >> Kpy >> Kpz >> Kox >> Koy >> Koz >> Kp_nullspace >> Dpx >> Dpy >> Dpz >> Dox >> Doy >> Doz >> Ipx >> Ipy >> Ipz >> Iox >> Ioy >> Ioz;
  }
  else{
    std::cout << "Error open the file!" << std::endl;
  }
  file_gains.close();

  // Damping ratio = 1
  // Dpx = 2.0 * sqrt(Kpx);
  // Dpy = 2.0 * sqrt(Kpy);
  // Dpz = 2.0 * sqrt(Kpz);
  // Dox = 2.0 * sqrt(Kox);
  // Doy = 2.0 * sqrt(Koy);
  // Doz = 2.0 * sqrt(Koz);

  // position stiffness in desired frame
  Kp_d_ << Kpx,   0,   0,
             0, Kpy,   0,
             0,   0, Kpz;

  // orientation stiffness in desired frame
  Ko_d_ << Kox,   0,   0,
             0, Koy,   0,
             0,   0, Koz;

  // position damping in desired frame
  Dp_d_ << Dpx,   0,   0,
             0, Dpy,   0,
             0,   0, Dpz;

  // orientation damping in desired frame
  Do_d_ << Dox,   0,   0,
             0, Doy,   0,
             0,   0, Doz;

  // position integral in desired frame
  Ip_d_ << Ipx,   0,   0,
            0, Ipy,   0,
            0,   0, Ipz;

  // orientation integral in desired frame
  Io_d_ << Iox,   0,   0,
            0, Ioy,   0,
            0,   0, Ioz;

  // nullspace Gains
  nullspace_stiffness_ = Kp_nullspace * nullspace_stiffness_.setIdentity();

}


void CartesianImpedanceController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

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
  Eigen::Map<Eigen::Matrix<double, 6, 1>> EE_force(robot_state.K_F_ext_hat_K.data()); // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame
  Eigen::Map<Eigen::Matrix<double, 6, 1>> O_force(robot_state.O_F_ext_hat_K.data()); // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame


  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix3d R(transform.rotation());

  // ---------------------------------------------------------------------------
  // Set the controller gains
  // ---------------------------------------------------------------------------
  Eigen::Matrix3d Kp(R_d_ * Kp_d_ * R_d_.transpose());  // cartesian position stiffness
  Eigen::Matrix3d Dp(R_d_ * Dp_d_ * R_d_.transpose());  // cartesian position damping
  Eigen::Matrix3d Ko(R_d_ * Ko_d_ * R_d_.transpose());  // cartesian orientation stiffness
  Eigen::Matrix3d Do(R_d_ * Do_d_ * R_d_.transpose());  // cartesian orientation damping
  Eigen::Matrix3d Ip(R_d_ * Ip_d_ * R_d_.transpose());  // cartesian position integrative
  Eigen::Matrix3d Io(R_d_ * Io_d_ * R_d_.transpose());  // cartesian position integrative

  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner(3, 3) << Kp;
  cartesian_stiffness_.bottomRightCorner(3, 3) << Ko;
  // std::cout << "\n" << cartesian_stiffness_ << std::endl;

  cartesian_damping_.setIdentity();
  cartesian_damping_.topLeftCorner(3, 3) << Dp;
  cartesian_damping_.bottomRightCorner(3, 3) << Do;
  //std::cout << "\n" << cartesian_damping_ << std::endl;

  cartesian_integral_.setIdentity();
  cartesian_integral_.topLeftCorner(3, 3) << Ip;
  cartesian_integral_.bottomRightCorner(3, 3) << Io;
  // std::cout << "\n" << cartesian_integral_ << std::endl;

  // ---------------------------------------------------------------------------
  // compute error to desired pose
  // ---------------------------------------------------------------------------
  // position error
  error.head(3) << position_d_ - position;

  // orientation error
  Eigen::Matrix3d cRcd(R.inverse() * R_d_); // described in current end-effector frame
  Eigen::Matrix3d Rcd(R * cRcd * R.inverse()); // described in the base frame
  error.tail(3) << R2r(Rcd);

  // velocity error
  Eigen::Matrix<double, 6, 1> velocity(jacobian * dq);
  velocity_error << velocity_d_ - velocity;

  // integral position error
  integral_error.head(3) = last_integral_error.head(3) + error.head(3);
  // establish limits (saturarion zone)
  for(int i = 0; i<3; i++){
    if(integral_error.head(3)[i] > 150.0){
      integral_error.head(3)[i] = 150.0;
    }
    else if(integral_error.head(3)[i] < -150.0){
      integral_error.head(3)[i] = -150.0;
    }
  }

  // integral orientation error
  double small_number = 0.000001;
  if( (error.tail(3).transpose() * error.tail(3)) < small_number ){
    integral_error.tail(3) = error.tail(3);
  }
  else{
    integral_error.tail(3) = ( (last_integral_error.tail(3).transpose() * error.tail(3)/error.tail(3).norm()).value() * error.tail(3)/error.tail(3).norm() ) + error.tail(3);
  }
  // establish limits (saturarion zone)
  for(int i = 0; i<3; i++){
    if(integral_error.tail(3)[i] > 100.0){
      integral_error.tail(3)[i] = 100.0;
    }
    else if(integral_error.tail(3)[i] < -100.0){
      integral_error.tail(3)[i] = -100.0;
    }
  }

  // ---------------------------------------------------------------------------
  // compute control
  // ---------------------------------------------------------------------------
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * ( cartesian_damping_ * velocity_error + cartesian_stiffness_ * error + cartesian_integral_ * integral_error );

  // compute the inertia matrix of the task space
  Eigen::Matrix<double, 6, 6> lambda( (jacobian * mass.inverse() * jacobian.transpose()).inverse() );

  // Dynamically consistent generalized inverse of the jacobian
  Eigen::Matrix<double, 7, 6> jacobian_dcgi = mass.inverse() * jacobian.transpose() * lambda;

  // The performance optimization torque
  gradient_mechanical_joint_limit<7>(gradient, q, maxJointLimits, minJointLimits);
  Eigen::Matrix<double, 7, 1>  tau_o( mass * nullspace_stiffness_ * gradient );

  // nullspace controll for posture optimization
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_dcgi.transpose()) * tau_o;

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
  // Quaternion_d_.coeffs() = filter_params_ * Quaternion_d_target_.coeffs() + (1.0 - filter_params_) * Quaternion_d_.coeffs();
  Quaternion_d_ = Quaternion_d_.slerp(filter_params_, Quaternion_d_target_);
  R_d_ = Quaternion_d_.toRotationMatrix();

  last_integral_error = integral_error; // update last integral error

  // ---------------------------------------------------------------------------
  // Write to file
  // ---------------------------------------------------------------------------
  // Eigen::Vector3d euler_angles(R.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)
  // Eigen::Vector3d euler_angles_d_(R_d_.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)
  Eigen::Vector3d euler_angles(R2EulerAngles(R)); // XYZ
  Eigen::Vector3d euler_angles_d_(R2EulerAngles(R_d_)); // XYZ

  Eigen::Vector3d position_EE(R*position); // current position in EE frame
  Eigen::Vector3d position_EE_d_(R_d_*position_d_); // current position in EE frame

  count++;
  double TIME = count/1000.0;
  file_tracking << " " << TIME << " "
                << position[0] << " " << position_d_[0] << " "
                << position[1] << " " << position_d_[1] << " "
                << position[2] << " " << position_d_[2] << " "
                << wrapTo2PI(euler_angles[0]) << " " << wrapTo2PI(euler_angles_d_[0]) << " "
                << wrapToPI(euler_angles[1]) << " " << wrapToPI(euler_angles_d_[1]) << " "
                << wrapToPI(euler_angles[2]) << " " << wrapToPI(euler_angles_d_[2]) << " "
                << EE_force[0] << " "
                << EE_force[1] << " "
                << EE_force[2] << " "
                << O_force[0] << " "
                << O_force[1] << " "
                << O_force[2] << " "
                << error[0] << " "
                << error[1] << " "
                << error[2] << " "
                << error[3] << " "
                << error[4] << " "
                << error[5] << " "
                << position_EE[0] << " " << position_EE_d_[0] << " "
                << position_EE[1] << " " << position_EE_d_[1] << " "
                << position_EE[2] << " " << position_EE_d_[2] << " "
                << integral_error[0] << " "
                << integral_error[1] << " "
                << integral_error[2] << " "
                << integral_error[3] << " "
                << integral_error[4] << " "
                << integral_error[5] << "\n";


}


Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                                                             const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }

  return tau_d_saturated;
}


Eigen::Vector3d CartesianImpedanceController::R2r(Eigen::Matrix3d& Rotation){
  Eigen::Vector3d aux, rotation_vector;
  aux << Rotation(2,1) - Rotation(1,2),
         Rotation(0,2) - Rotation(2,0),
         Rotation(1,0) - Rotation(0,1);
  rotation_vector = 0.5 * aux;
  return rotation_vector;
}


Eigen::Vector3d CartesianImpedanceController::R2EulerAngles(Eigen::Matrix3d& Rotation){
  Eigen::Vector3d XYZ;
  double sy = sqrt( Rotation(0,0) * Rotation(0,0) +  Rotation(1,0) * Rotation(1,0) );
  bool singular = sy < 1e-6; // If
  double x, y, z;
  if (!singular){
      x = atan2(Rotation(2,1) , Rotation(2,2));
      y = atan2(-Rotation(2,0), sy);
      z = atan2(Rotation(1,0), Rotation(0,0));
  }
  else{
      x = atan2(-Rotation(1,2), Rotation(1,1));
      y = atan2(-Rotation(2,0), sy);
      z = 0;
  }
  XYZ << x, y, z;
  return XYZ;
}


double CartesianImpedanceController::wrapToPI(double& angle){
  double new_angle = atan2(sin(angle), cos(angle));
  return new_angle;
}


double CartesianImpedanceController::wrapTo2PI(double& angle){
  double new_angle = asin(sin(angle));
  if(cos(angle) < 0){
    new_angle = M_PI-new_angle;
  }
  else if(new_angle < 0){
    new_angle += 2*M_PI;
  }
  return new_angle;
}


void CartesianImpedanceController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  position_d_target_ << msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z;
  Eigen::Quaterniond last_Quaternion_d_target(Quaternion_d_target_);
  Quaternion_d_target_.coeffs() << msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w;
  if (last_Quaternion_d_target.coeffs().dot(Quaternion_d_target_.coeffs()) < 0.0){
    Quaternion_d_target_.coeffs() << -Quaternion_d_target_.coeffs();
  }
}


double CartesianImpedanceController::derivative_computation( const double q_i, const double maxJointLimit_i, const double minJointLimit_i){
  double result;
  double average_joint;
  average_joint = ( maxJointLimit_i + minJointLimit_i) / 2.0;
  result = - ( ( ( q_i - average_joint ) / pow( ( maxJointLimit_i - minJointLimit_i ), 2 ) ) );

  return result;
}

template<int N> // number of joints or DOF
void CartesianImpedanceController::gradient_mechanical_joint_limit( Eigen::Matrix<double, N, 1>& gradient_mechanical_joint_limit_out, const Eigen::Matrix<double, N, 1> q, const Eigen::Matrix<double, N, 1> maxJointLimits, const Eigen::Matrix<double, N, 1> minJointLimits ){
  for ( int i = 0; i < q.rows(); i++ ){
    gradient_mechanical_joint_limit_out(i) = derivative_computation( q(i), maxJointLimits(i), minJointLimits(i) );
  }
}


} // namespace franka_polishing

PLUGINLIB_EXPORT_CLASS(franka_polishing::CartesianImpedanceController,
                       controller_interface::ControllerBase)
