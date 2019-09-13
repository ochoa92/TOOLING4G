#include <franka_simulation/cartesian_impedance_controller.h>

namespace franka_simulation {


CartesianImpedanceController::CartesianImpedanceController(){
  std::cout << "Open the file to write!" << std::endl;
  std::string tracking_path;
  tracking_path = "/home/helio/kst/simulation/cartesian_impedance_controller";
  file_tracking.open(tracking_path, std::ofstream::out);
  file_tracking << " t p_x p_xd p_y p_yd p_z p_zd Yaw(X) Yaw_d(Xd) Pitch(Y) Pitch_d(Yd) Roll(Z) Roll_d(Zd) e_px e_py e_pz e_ox e_oy e_oz\n";
  file_tracking << " s m m m m m m rad rad rad rad rad rad m m m rad rad rad\n";
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

  // read URDF from param server
  if(!kdl_parser::treeFromParam("robot_description", kdl_tree)){
    ROS_ERROR("Failed to construct kdl tree!");
  }

  // Get root and end effector from parameter server
  root_name = "panda_link0";
  end_effector_name = "panda_hand";

  // Get chain from kdl tree
  if(!kdl_tree.getChain(root_name, end_effector_name, kdl_chain)){
    ROS_ERROR("Failed to get chain from kdl tree!");
  }

  // Get number of joints
  n_joints = kdl_chain.getNrOfJoints();

  spacenav_sub = node_handle.subscribe("/spacenav/joy", 20, &CartesianImpedanceController::joy_callback, this);

  // ---------------------------------------------------------------------------
  // Init Values
  // ---------------------------------------------------------------------------
  O_T_EE_d.setZero();
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  nullspace_stiffness_.setZero();
  position_d_.setZero();
  R_d_.setZero();
  velocity_d_.setZero();
  error.setZero();
  velocity_error.setZero();
  spacenav_motion.setZero();
  T_spacenav.setZero();
  Tx.setZero();
  Ty.setZero();
  Tz.setZero();
  maxJointLimits << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  minJointLimits << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  gradient.setZero();

  count = 0;

  return true;
}


void CartesianImpedanceController::starting(const ros::Time& /*time*/) {

  // O_T_EE_d <<   0.498596, -0.0385043,   0.865979,    0.396799,
  //             -0.0133706,  -0.999236, -0.0367311, -0.00644199,
  //               0.866732, 0.00673532,  -0.498729,    0.490258,
  //                      0,          0,          0,           1;
  q_start << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
  FK(forward_kinematics, q_start);
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++){
      O_T_EE_d(i,j) = forward_kinematics(i,j);
    }
  }
  Eigen::Affine3d initial_transform(O_T_EE_d);
  position_d_ = initial_transform.translation();
  R_d_ = initial_transform.rotation();

  // ---------------------------------------------------------------------------
  // Get the controller gains from a file
  // ---------------------------------------------------------------------------
  std::string path_gains;
  path_gains = "/home/helio/catkin_ws/src/TOOLING4G/franka_simulation/controller_gains/compliance_param";
  file_gains.open(path_gains);
  if(file_gains.is_open()){
    file_gains >> Kpx >> Kpy >> Kpz >> Kox >> Koy >> Koz >> Kp_nullspace;
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

  // For a non-linear system
  Dpx = 100.0;
  Dpy = 100.0;
  Dpz = 100.0;
  Dox = 10.0;
  Doy = 10.0;
  Doz = 10.0;

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

  // nullspace Gains
  nullspace_stiffness_ = Kp_nullspace * nullspace_stiffness_.setIdentity();

}


void CartesianImpedanceController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

  // ---------------------------------------------------------------------------
  // get state variables
  // ---------------------------------------------------------------------------
  for (size_t i = 0; i < 7; ++i) {
    q(i) = joint_handles_[i].getPosition();
    dq(i) = joint_handles_[i].getVelocity();
    effort(i) = joint_handles_[i].getEffort();
  }

  FK(forward_kinematics, q);
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++){
      O_T_EE(i,j) = forward_kinematics(i,j);
    }
  }
  Eigen::Affine3d transform(O_T_EE);
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix3d R(transform.rotation());

  jacobian(jac, q);
  for (int i = 0; i < 6; i++){
    for (int j = 0; j < n_joints; j++){
      J(i,j) = jac(i,j);
    }
  }

  dynamic(inertia, coriolis, gravity, q, dq, g_vector);  // Update robotics dynamics
  for (int i = 0; i < n_joints; i++){
    for (int j = 0; j < n_joints; j++){
      M(i,j) = inertia(i,j);
      C(i) = coriolis(i);
      g(i) = gravity(i);
    }
  }


  // ---------------------------------------------------------------------------
  // Set the controller gains
  // ---------------------------------------------------------------------------
  Eigen::Matrix3d Kp(R_d_ * Kp_d_ * R_d_.transpose());  // cartesian position stiffness
  Eigen::Matrix3d Dp(R_d_ * Dp_d_ * R_d_.transpose());  // cartesian position damping
  Eigen::Matrix3d Ko(R_d_ * Ko_d_ * R_d_.transpose());  // cartesian orientation stiffness
  Eigen::Matrix3d Do(R_d_ * Do_d_ * R_d_.transpose());  // cartesian orientation damping

  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner(3, 3) << Kp;
  cartesian_stiffness_.bottomRightCorner(3, 3) << Ko;
  //std::cout << "\n" << cartesian_stiffness_ << std::endl;

  cartesian_damping_.setIdentity();
  cartesian_damping_.topLeftCorner(3, 3) << Dp;
  cartesian_damping_.bottomRightCorner(3, 3) << Do;
  //std::cout << "\n" << cartesian_damping_ << std::endl;


  // ---------------------------------------------------------------------------
  // compute error
  // ---------------------------------------------------------------------------
  // position error
  error.head(3) << position_d_ - position;

  // orientation error
  Eigen::Matrix3d cRcd(R.inverse() * R_d_); // described in current end-effector frame
  Eigen::Matrix3d Rcd(R * cRcd * R.inverse()); // described in the base frame
  error.tail(3) << R2r(Rcd);

  // velocity error
  Eigen::Matrix<double, 6, 1> velocity(J * dq);
  velocity_error << velocity_d_ - velocity;

  // ---------------------------------------------------------------------------
  // compute control
  // ---------------------------------------------------------------------------
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_d(7), tau_nullspace(7);

  // compute the inertia matrix of the task space
  Eigen::Matrix<double, 6, 6> lambda( (J * M.inverse() * J.transpose()).inverse() );

  // Cartesian PD control with damping ratio = 1
  // tau_task << J.transpose() * lambda * ( cartesian_damping_ * velocity_error + cartesian_stiffness_ * error );
  tau_task << J.transpose() * ( cartesian_damping_ * velocity_error + cartesian_stiffness_ * error );

  // Dynamically consistent generalized inverse of the jacobian
  Eigen::Matrix<double, 7, 6> J_dcgi = M.inverse() * J.transpose() * lambda;

  // The performance optimization torque
  gradient_mechanical_joint_limit<7>(gradient, q, maxJointLimits, minJointLimits);
  Eigen::Matrix<double, 7, 1>  tau_o( M * nullspace_stiffness_ * gradient );

  // nullspace controll for posture optimization
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - J.transpose() * J_dcgi.transpose()) * tau_o;

  // Desired torque
  tau_d << tau_task + tau_nullspace + C + g;
  // std::cout << tau_d << std::endl;

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // ---------------------------------------------------------------------------
  // update parameters
  // ---------------------------------------------------------------------------
  MotionControl(O_T_EE_d);

  // ---------------------------------------------------------------------------
  // Write to file
  // ---------------------------------------------------------------------------
  // Eigen::Vector3d euler_angles(R.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)
  // Eigen::Vector3d euler_angles_d_(R_d_.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)
  Eigen::Vector3d euler_angles(R2EulerAngles(R)); // XYZ
  Eigen::Vector3d euler_angles_d_(R2EulerAngles(R_d_)); // XYZ

  count++;
  double TIME = count/1000.0;
  file_tracking << " " << TIME << " "
                << position[0] << " " << position_d_[0] << " "
                << position[1] << " " << position_d_[1] << " "
                << position[2] << " " << position_d_[2] << " "
                << euler_angles[0] << " " << euler_angles_d_[0] << " "
                << euler_angles[1] << " " << euler_angles_d_[1] << " "
                << euler_angles[2] << " " << euler_angles_d_[2] << " "
                << error[0] << " "
                << error[1] << " "
                << error[2] << " "
                << error[3] << " "
                << error[4] << " "
                << error[5] << "\n";

}


Eigen::Vector3d CartesianImpedanceController::R2r(Eigen::Matrix3d& Rotation){
  Eigen::Vector3d rotation_vector, aux;
  aux << Rotation(2,1) - Rotation(1,2),
         Rotation(0,2) - Rotation(2,0),
         Rotation(1,0) - Rotation(0,1);
  rotation_vector = 0.5 * aux;

  return rotation_vector;
}


Eigen::Vector3d CartesianImpedanceController::R2EulerAngles(Eigen::Matrix3d& Rotation){
  Eigen::Vector3d euler_angles;
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
  euler_angles << x, y, z;

  return euler_angles;
}


bool CartesianImpedanceController::FK(KDL::Frame& kdl_frame, Eigen::Matrix<double, 7, 1>& q_values){
  if (q_values.size() != n_joints){
    return false;
  }

  // Create KDL FK Solver
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(kdl_chain);

  // Create Joint Array for calculations
  KDL::JntArray joint_position_array = JntArray(n_joints);

  for (int i = 0; i < n_joints; i++){
    joint_position_array(i) = q_values[i];
  }

  fksolver.JntToCart(joint_position_array, kdl_frame);

  return true;
}


bool CartesianImpedanceController::jacobian(KDL::Jacobian &kdl_jacobian, Eigen::Matrix<double, 7, 1> q_values){
  if(q_values.size() != n_joints){
    return false;
  }
  else{
    // create KDL Jacobian Solver
    ChainJntToJacSolver jac_solver = ChainJntToJacSolver(kdl_chain);

    // create Joint Array for calculations
    KDL::JntArray joint_position_array = JntArray(n_joints);

    // define Jacobian with the correct number of columns
    kdl_jacobian = Jacobian(n_joints);

    for(int i=0; i<n_joints; i++){
      joint_position_array(i) = q_values[i];
    }

    jac_solver.JntToJac(joint_position_array, kdl_jacobian);

    return true;
  }
}


bool CartesianImpedanceController::dynamic(KDL::JntSpaceInertiaMatrix& kdl_inertia, KDL::JntArray& kdl_coriolis, KDL::JntArray& kdl_gravity, Eigen::Matrix<double, 7, 1>& q_values, Eigen::Matrix<double, 7, 1>& dq_values, Vector& g_vector){
  if(q_values.size() != n_joints || dq_values.size() != n_joints){
    return false;
  }
  else{
    // create KDL Dynamic Solver
    ChainDynParam dyn_solver = ChainDynParam(kdl_chain, g_vector);

    // create Joint Array for calculations
    KDL::JntArray joint_position_array = JntArray(n_joints);
    KDL::JntArray joint_velocity_array = JntArray(n_joints);

    // define inertia matrix with correct size (rows and columns)
    kdl_inertia = JntSpaceInertiaMatrix(n_joints);
    kdl_coriolis = JntArray(n_joints);
    kdl_gravity = JntArray(n_joints);

    for(int i=0; i<n_joints; i++){
      joint_position_array(i) = q_values[i];
      joint_velocity_array(i) = dq_values[i];
    }

    dyn_solver.JntToMass(joint_position_array, kdl_inertia);
    dyn_solver.JntToCoriolis(joint_position_array, joint_velocity_array, kdl_coriolis);
    dyn_solver.JntToGravity(joint_position_array, kdl_gravity);

    return true;
  }
}


void CartesianImpedanceController::joy_callback(const sensor_msgs::Joy::ConstPtr &msg){

  double threshold = 0.5, K = 0;
  ros::Duration d(0.5);

  for (int i = 0; i < 6; i++){
    if (msg->axes[i] >= threshold || msg->axes[i] <= -threshold)
    {
      if (i >= 0 && i <= 2)
        K = 0.0005;
      else if (i >= 3 && i <= 5)
        K = 0.0005;

      spacenav_motion(i) = K * msg->axes[i];
    }

    else if (msg->axes[i] <= threshold || msg->axes[i] >= -threshold)
      spacenav_motion(i) = 0;
  }

  time_lapse = ros::Time::now();
  if (msg->buttons[0] && time_lapse - last_time > d){
    last_time = time_lapse;
    if (flag_mode == 0)
      flag_mode = 1;
    else if (flag_mode == 1)
      flag_mode = 2;
    else if (flag_mode == 2)
      flag_mode = 0;
  }
}


void CartesianImpedanceController::MotionControl(Eigen::Matrix4d &Xd){

  Eigen::Matrix4d last_Xd;

  Eigen::Matrix3d last_R_d;
  Eigen::Matrix3d R_spacenav;
  Eigen::Matrix3d R_EE; // rotation in End-Effector frame
  Eigen::Vector3d last_position_d;
  Eigen::Vector3d position_spacenav;
  Eigen::Vector3d position_B;  // position in base frame

  Eigen::Affine3d aff_d;
  Eigen::Affine3d last_aff_d;
  Eigen::Affine3d aff_spacenav;

  for (int i = 0; i < 6; i++){
    if (spacenav_motion(i) != 0){
      flag_motion = true;
      break;
    }

    else if(spacenav_motion(i) == 0)
      continue;
  }

  if (flag_motion == false){
    T_spacenav << 1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1;
  }

  else if (flag_motion){
    flag_motion = false;

    Tx << 1,                       0,                        0, -spacenav_motion(0),
          0, cos(spacenav_motion(3)), -sin(spacenav_motion(3)),                  0,
          0, sin(spacenav_motion(3)),  cos(spacenav_motion(3)),                  0,
          0,                       0,                        0,                  1;

    Ty <<  cos(spacenav_motion(4)), 0, sin(spacenav_motion(4)),                  0,
                                 0, 1,                       0, -spacenav_motion(1),
          -sin(spacenav_motion(4)), 0, cos(spacenav_motion(4)),                  0,
                                 0, 0,                       0,                  1;

    Tz << cos(spacenav_motion(5)), -sin(spacenav_motion(5)), 0,                  0,
          sin(spacenav_motion(5)),  cos(spacenav_motion(5)), 0,                  0,
                                0,                        0, 1, spacenav_motion(2),
                                0,                        0, 0,                  1;

    T_spacenav = Tx * Ty * Tz;
    //T_spacenav = Tz * Ty * Tx;
  }

  last_Xd = Xd;

  switch (flag_mode){

    case 0:
      if(flag_print == 0){
        std::cout << CLEANWINDOW << "OPERATING ROTATION IN END-EFFECTOR AND TRANSLATION IN BASE..." << std::endl;
        flag_print = 1;
      }

      // rotation in EE frame
      last_aff_d.matrix() = last_Xd;
      last_R_d = last_aff_d.rotation();

      aff_spacenav.matrix() = T_spacenav;
      R_spacenav = aff_spacenav.rotation();

      R_EE = last_R_d * R_spacenav;

      // position in Base frame
      last_position_d = last_aff_d.translation();
      position_spacenav = aff_spacenav.translation();

      position_B = position_spacenav + last_position_d;


      Xd << R_EE(0,0), R_EE(0,1), R_EE(0,2), position_B(0),
            R_EE(1,0), R_EE(1,1), R_EE(1,2), position_B(1),
            R_EE(2,0), R_EE(2,1), R_EE(2,2), position_B(2),
                    0,         0,         0,             1;

      aff_d.matrix() = Xd;
      position_d_ = aff_d.translation();
      R_d_ = aff_d.rotation();

      break;

    case 1:
      if(flag_print == 1){
        std::cout << CLEANWINDOW << "OPERATING ROTATION IN BASE..." << std::endl;
        flag_print = 2;
      }

      Xd = T_spacenav * last_Xd;

      aff_d.matrix() = Xd;
      position_d_ = aff_d.translation();
      R_d_ = aff_d.rotation();

      break;

    case 2:
      if(flag_print == 2){
        std::cout << CLEANWINDOW << "OPERATING ROTATION IN END-EFFECTOR..." << std::endl;
        flag_print = 0;
      }

      Xd = last_Xd * T_spacenav;

      aff_d.matrix() = Xd;
      position_d_ = aff_d.translation();
      R_d_ = aff_d.rotation();

      break;

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


} // namespace franka_simulation

PLUGINLIB_EXPORT_CLASS(franka_simulation::CartesianImpedanceController,
                       controller_interface::ControllerBase)
