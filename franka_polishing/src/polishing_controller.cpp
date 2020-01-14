#include <franka_polishing/polishing_controller.h>

namespace franka_polishing {

// ----------------------------------------------------------------------------
PolishingController::PolishingController(){
    // std::cout << "\nOpen the tracking file to write!" << std::endl << std::endl;
    // tracking_file.open("/home/panda/kst/polishing/polishing_controller", std::ofstream::out);
    // tracking_file << "t p_x p_xd p_y p_yd p_z p_zd Yaw Yaw_d Pitch Pitch_d Roll Roll_d Fx_EE Fy_EE Fz_EE Fx_O Fy_O Fz_O e_px e_py e_pz e_ox e_oy e_oz pEE_x pEE_xd pEE_y pEE_yd pEE_z pEE_zd i_px i_py i_pz i_ox i_oy i_oz\n";
    // tracking_file << "s m m m m m m rad rad rad rad rad rad N N N N N N m m m rad rad rad m m m m m m m m m rad rad rad\n";
}

// ----------------------------------------------------------------------------
PolishingController::~PolishingController(){
    // std::cout << "\nTracking file closed!" << std::endl << std::endl;
    // tracking_file.close();
}

// ----------------------------------------------------------------------------
bool PolishingController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle){

    std::string arm_id;
    if(!node_handle.getParam("arm_id", arm_id)){
        ROS_ERROR_STREAM("PolishingController: Could not read parameter arm_id");
        return false;
    }

    std::vector<std::string> joint_names;
    if(!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7){
        ROS_ERROR("PolishingController: Invalid or no joint_names parameters provided, aborting controller init!");
        return false;
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if(model_interface == nullptr){
        ROS_ERROR_STREAM("PolishingController: Error getting model interface from hardware");
        return false;
    }
    try{
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    }catch(hardware_interface::HardwareInterfaceException& ex){
        ROS_ERROR_STREAM("PolishingController: Exception getting model handle from interface: " << ex.what());
        return false;
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if(state_interface == nullptr){
        ROS_ERROR_STREAM("PolishingController: Error getting state interface from hardware");
        return false;
    }
    try{
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    }catch(hardware_interface::HardwareInterfaceException& ex){
        ROS_ERROR_STREAM("PolishingController: Exception getting state handle from interface: " << ex.what());
        return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if(effort_joint_interface == nullptr){
        ROS_ERROR_STREAM("PolishingController: Error getting effort joint interface from hardware");
        return false;
    }
    for(size_t i = 0; i < 7; ++i){
        try{
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        }catch(const hardware_interface::HardwareInterfaceException& ex){
            ROS_ERROR_STREAM("PolishingController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    sub_equilibrium_pose_ = node_handle.subscribe("/panda_equilibrium_pose", 20, &PolishingController::equilibriumPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());

    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle("dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique< dynamic_reconfigure::Server<franka_polishing::compliance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&PolishingController::complianceParamCallback, this, _1, _2));

    // init values
    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    R_d_.setIdentity();

    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    velocity_d_.setZero();

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();
    cartesian_integral_.setZero();
    nullspace_stiffness_.setZero();

    Kp_d_.setZero();
    Dp_d_.setZero();

    Ko_d_.setZero();
    Do_d_.setZero();

    Ip_d_.setZero();
    Io_d_.setZero();

    error.setZero();
    velocity_error.setZero();
    integral_error.setZero();
    last_integral_error.setZero();

    maxJointLimits << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    minJointLimits << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    gradient.setZero();

    K_external_torque_.setZero();
    K_external_torque_target_.setZero();

    count = 0;

    // Create pose publishers
    poseEE_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/robot_poseEE", 20);
    poseEE_d_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/robot_poseEE_d", 20);

    // Create error publisher
    error_pub = node_handle.advertise<geometry_msgs::Twist>("/robot_error", 20);

    return true;
}

// ----------------------------------------------------------------------------
void PolishingController::starting(const ros::Time& /*time*/){

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
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
    orientation_d_.normalize();

    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
    orientation_d_target_.normalize();

    // ---------------------------------------------------------------------------
    // Get mold pose from a file
    // ---------------------------------------------------------------------------
    std::ifstream ws_file;
    double x, y, z;
    Eigen::MatrixXd P;
    ws_file.open("/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/mold_workspace");
    int n_ws = 0;
    P.resize(n_ws + 1, 3);
    if(ws_file.is_open()){
        while(ws_file >> x >> y >> z){
            // save the values in the matrix
            P.conservativeResize(n_ws + 1, 3);
            P(n_ws, 0) = x;
            P(n_ws, 1) = y;
            P(n_ws, 2) = z;
            n_ws++;
        }
    }
    else{
        std::cout << "Error open the file!" << std::endl;
    }
    ws_file.close();
    // std::cout << P << std::endl;

    P1 = P.row(0);
    P2 = P.row(1);
    P3 = P.row(2);
    P4 = P.row(3);

}

// ----------------------------------------------------------------------------
void PolishingController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

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
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_ext_hat_filtered.data()); // External torque, filtered.


    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    Eigen::Matrix3d R(transform.rotation());


    // ---------------------------------------------------------------------------
    // Set the impedance controller gains
    // ---------------------------------------------------------------------------
    Eigen::Matrix3d Kp(R_d_ * Kp_d_ * R_d_.transpose());  // cartesian position stiffness
    Eigen::Matrix3d Ko(R_d_ * Ko_d_ * R_d_.transpose());  // cartesian orientation stiffness
    Eigen::Matrix3d Dp(R_d_ * Dp_d_ * R_d_.transpose());  // cartesian position damping
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
    // std::cout << "\n" << cartesian_damping_ << std::endl;

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

    // Cartesian Impedance control
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

    tau_measured << K_external_torque_ * tau_measured;
    // std::cout << "\n" << tau_measured << std::endl;
    // Eigen::Matrix<double, 6, 1> F_measured( jacobian_dcgi.transpose() * tau_measured );
    // std::cout << "\n" << F_measured(2) << std::endl;

    // Desired torque
    // tau_d << tau_task + tau_nullspace + coriolis;
    tau_d << tau_task + tau_nullspace + coriolis - tau_measured;
    // std::cout << "\n" << tau_d << std::endl;

    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].setCommand(tau_d(i));
    }

    // ---------------------------------------------------------------------------
    // update parameters (target by filtering)
    // ---------------------------------------------------------------------------
    // panda equilibrium pose
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
    orientation_d_.normalize();
    R_d_ = orientation_d_.toRotationMatrix();

    // compliance parameter gains
    Kp_d_ = filter_params_ * Kp_d_target_ + (1.0 - filter_params_) * Kp_d_;
    Ko_d_ = filter_params_ * Ko_d_target_ + (1.0 - filter_params_) * Ko_d_;
    Dp_d_ = filter_params_ * Dp_d_target_ + (1.0 - filter_params_) * Dp_d_;
    Do_d_ = filter_params_ * Do_d_target_ + (1.0 - filter_params_) * Do_d_;
    Ip_d_ = filter_params_ * Ip_d_target_ + (1.0 - filter_params_) * Ip_d_;
    Io_d_ = filter_params_ * Io_d_target_ + (1.0 - filter_params_) * Io_d_;

    nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    // std::cout << "\n" << nullspace_stiffness_ << std::endl;

    K_external_torque_ = filter_params_ * K_external_torque_target_ + (1.0 - filter_params_) * K_external_torque_;
    // std::cout << "\n" << K_external_torque_ << std::endl;

    // update last integral error
    last_integral_error = integral_error;

    // update pose publishers
    posePublisherCallback(poseEE_pub, position, orientation);
    posePublisherCallback(poseEE_d_pub, position_d_, orientation_d_);

    // update error publisher
    errorPublisherCallback(error_pub, error);

    // publish mold frame
    Eigen::Matrix3d Rmold(points2Rotation(P1, P2, P4));
    Eigen::Vector3d mold_position(P1);
    Eigen::Quaterniond mold_orientation(Rmold);
    publishFrame(br_mold, tf_mold, mold_position, mold_orientation, "/panda_link0", "/polishing_mold");

    // ---------------------------------------------------------------------------
    // Write to file
    // ---------------------------------------------------------------------------
    Eigen::Vector3d euler_angles(R.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)
    Eigen::Vector3d euler_angles_d_(R_d_.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)

    Eigen::Vector3d position_EE(R*position); // current position in EE frame
    Eigen::Vector3d position_EE_d_(R_d_*position_d_); // current position in EE frame

    count++;
    // double TIME = count/1000.0;
    // tracking_file << TIME << " "
    //               << position[0] << " " << position_d_[0] << " "
    //               << position[1] << " " << position_d_[1] << " "
    //               << position[2] << " " << position_d_[2] << " "
    //               << wrapTo2PI(euler_angles[0]) << " " << wrapTo2PI(euler_angles_d_[0]) << " "
    //               << wrapTo2PI(euler_angles[1]) << " " << wrapTo2PI(euler_angles_d_[1]) << " "
    //               << wrapToPI(euler_angles[2]) << " " << wrapToPI(euler_angles_d_[2]) << " "
    //               << EE_force[0] << " "
    //               << EE_force[1] << " "
    //               << EE_force[2] << " "
    //               << O_force[0] << " "
    //               << O_force[1] << " "
    //               << O_force[2] << " "
    //               << error[0] << " "
    //               << error[1] << " "
    //               << error[2] << " "
    //               << error[3] << " "
    //               << error[4] << " "
    //               << error[5] << " "
    //               << position_EE[0] << " " << position_EE_d_[0] << " "
    //               << position_EE[1] << " " << position_EE_d_[1] << " "
    //               << position_EE[2] << " " << position_EE_d_[2] << " "
    //               << integral_error[0] << " "
    //               << integral_error[1] << " "
    //               << integral_error[2] << " "
    //               << integral_error[3] << " "
    //               << integral_error[4] << " "
    //               << integral_error[5] << "\n";

    // std::cout << "control_command_success_rate" << robot_state.control_command_success_rate << std::endl;

}

// ----------------------------------------------------------------------------
Eigen::Matrix<double, 7, 1> PolishingController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                                                    const Eigen::Matrix<double, 7, 1>& tau_J_d){  // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for(size_t i = 0; i < 7; i++){
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
}

// ----------------------------------------------------------------------------
Eigen::Vector3d PolishingController::R2r(Eigen::Matrix3d& Rotation){
    Eigen::Vector3d aux, rotation_vector;
    aux << Rotation(2,1) - Rotation(1,2),
           Rotation(0,2) - Rotation(2,0),
           Rotation(1,0) - Rotation(0,1);
    rotation_vector = 0.5 * aux;
    return rotation_vector;
}

// ----------------------------------------------------------------------------
double PolishingController::wrapToPI(double& angle){
    double new_angle = atan2(sin(angle), cos(angle));
    return new_angle;
}

// ----------------------------------------------------------------------------
double PolishingController::wrapTo2PI(double& angle){
    double new_angle = asin(sin(angle));
    if(cos(angle) < 0){
        new_angle = M_PI-new_angle;
    }
    else if(new_angle < 0){
        new_angle += 2*M_PI;
    }
    return new_angle;
}

// ----------------------------------------------------------------------------
double PolishingController::derivative_computation( const double q_i, const double maxJointLimit_i, const double minJointLimit_i){
    double result;
    double average_joint;
    average_joint = ( maxJointLimit_i + minJointLimit_i) / 2.0;
    result = - ( ( ( q_i - average_joint ) / pow( ( maxJointLimit_i - minJointLimit_i ), 2 ) ) );

    return result;
}

// ----------------------------------------------------------------------------
template<int N> // number of joints or DOF
void PolishingController::gradient_mechanical_joint_limit( Eigen::Matrix<double, N, 1>& gradient_mechanical_joint_limit_out, const Eigen::Matrix<double, N, 1> q, const Eigen::Matrix<double, N, 1> maxJointLimits, const Eigen::Matrix<double, N, 1> minJointLimits ){
    for ( int i = 0; i < q.rows(); i++ ){
        gradient_mechanical_joint_limit_out(i) = derivative_computation( q(i), maxJointLimits(i), minJointLimits(i) );
    }
}

// ----------------------------------------------------------------------------
void PolishingController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){

    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
    if(last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0){
        orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }

}

// ----------------------------------------------------------------------------
void PolishingController::complianceParamCallback(franka_polishing::compliance_paramConfig& config, uint32_t /*level*/){

    // position stiffness in desired frame
    Kp_d_target_ << config.Kpx,          0,          0,
                             0, config.Kpy,          0,
                             0,          0, config.Kpz;

    // orientation stiffness in desired frame
    Ko_d_target_ << config.Kox,          0,          0,
                             0, config.Koy,          0,
                             0,          0, config.Koz;

    // position damping in desired frame
    Dp_d_target_ << config.Dpx,          0,          0,
                             0, config.Dpy,          0,
                             0,          0, config.Dpz;

    // orientation damping in desired frame
    Do_d_target_ << config.Dox,          0,          0,
                             0, config.Doy,          0,
                             0,          0, config.Doz;

    // position integral in desired frame
    Ip_d_target_ << config.Ipx,          0,          0,
                             0, config.Ipy,          0,
                             0,          0, config.Ipz;

    // orientation integral in desired frame
    Io_d_target_ << config.Iox,          0,          0,
                             0, config.Ioy,          0,
                             0,          0, config.Ioz;

    // nullspace stiffness target
    nullspace_stiffness_target_ = config.Kp_nullspace * nullspace_stiffness_target_.setIdentity();

    // external torque OFF/ON
    if(config.external_torque == 0){    // OFF
        K_external_torque_target_ << K_external_torque_target_.setZero();
    }
    else{   // ON
        K_external_torque_target_ << K_external_torque_target_.setIdentity();
    }

}

// ----------------------------------------------------------------------------
void PolishingController::posePublisherCallback(ros::Publisher& pose_pub, Eigen::Vector3d& position, Eigen::Quaterniond& orientation){

    geometry_msgs::PoseStamped robot_pose;

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

// ----------------------------------------------------------------------------
void PolishingController::errorPublisherCallback(ros::Publisher& error_pub, Eigen::Matrix<double, 6, 1>& error){

    geometry_msgs::Twist robot_error;

    robot_error.linear.x = error[0];
    robot_error.linear.y = error[1];
    robot_error.linear.z = error[2];

    robot_error.angular.x = error[3];
    robot_error.angular.y = error[4];
    robot_error.angular.z = error[5];

    // run error publisher
    error_pub.publish(robot_error);

}

// ----------------------------------------------------------------------------
void PolishingController::publishFrame(tf::TransformBroadcaster& br, tf::Transform& transform, Eigen::Vector3d& position, Eigen::Quaterniond& orientation, std::string base_link, std::string link_name){
    transform.setOrigin( tf::Vector3(position(0), position(1), position(2)) );
    transform.setRotation( tf::Quaternion(orientation.vec()[0], orientation.vec()[1], orientation.vec()[2], orientation.w()) );
    br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), base_link, link_name) );
}

// ----------------------------------------------------------------------------
Eigen::Matrix3d PolishingController::points2Rotation(Eigen::Vector3d& P1, Eigen::Vector3d& P2, Eigen::Vector3d& P3){

    // compute the vectors of the desired rotation matrix
    Eigen::Vector3d nx( (P3-P1)/((P3-P1).norm()) );
    Eigen::Vector3d ny( (P2-P1)/((P2-P1).norm()) );
    Eigen::Vector3d nz( nx.cross(ny) );
    nz = nz/(nz.norm());

    // complete the rotation matrix (orthogonality characteristic)
    ny = nz.cross(nx);
    ny = ny/(ny.norm());

    // construct the rotation matrix R = [nx', ny', nz']
    Eigen::Matrix3d R;
    R << nx(0), ny(0), nz(0),
         nx(1), ny(1), nz(1),
         nx(2), ny(2), nz(2);

    return R;
}


} // namespace franka_polishing

PLUGINLIB_EXPORT_CLASS(franka_polishing::PolishingController, controller_interface::ControllerBase)
