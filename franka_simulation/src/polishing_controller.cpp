#include <franka_simulation/polishing_controller.h>

namespace franka_simulation {


PolishingController::PolishingController(){
    std::cout << "Open the file to write!" << std::endl;
    std::string tracking_path;
    tracking_path = "/home/panda/kst/simulation/polishing_controller";
    file_tracking.open(tracking_path, std::ofstream::out);
    file_tracking << "t p_x p_xd p_y p_yd p_z p_zd Yaw(X) Yaw_d(Xd) Pitch(Y) Pitch_d(Yd) Roll(Z) Roll_d(Zd) e_px e_py e_pz e_ox e_oy e_oz i_px i_py i_pz i_ox i_oy i_oz Fx Fy Fz Fx_filtered Fy_filtered Fz_filtered\n";
    file_tracking << "s m m m m m m rad rad rad rad rad rad m m m rad rad rad m m m rad rad rad N N N N N N\n";
}

PolishingController::~PolishingController(){
    std::cout << "File closed!" << std::endl << std::endl;
    file_tracking.close();
}


bool PolishingController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle){

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("PolishingController: Could not read parameter arm_id");
        return false;
    }

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR("PolishingController: Invalid or no joint_names parameters provided, aborting controller init!");
        return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM("PolishingController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("PolishingController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    // read URDF from param server
    if(!kdl_parser::treeFromParam("robot_description", kdl_tree)){
        ROS_ERROR("Failed to construct kdl tree!");
    }
    
    // Get root and end effector from parameter server
    root_name = "panda_link0";
    end_effector_name = "panda_EE";

    // Get chain from kdl tree
    if(!kdl_tree.getChain(root_name, end_effector_name, kdl_chain)){
        ROS_ERROR("Failed to get chain from kdl tree!");
    }

    // Get number of joints
    n_joints = kdl_chain.getNrOfJoints();

    sub_equilibrium_pose_ = node_handle.subscribe("/panda_equilibrium_pose", 20, &PolishingController::equilibriumPoseCallback, this);

    poseEE_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/panda_poseEE", 20);

    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle("dynamic_reconfigure_compliance_param_node");
    dynamic_server_compliance_param_ = std::make_unique< dynamic_reconfigure::Server<franka_simulation::compliance_paramConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(boost::bind(&PolishingController::complianceParamCallback, this, _1, _2));

    // ---------------------------------------------------------------------------
    // Init Values
    // ---------------------------------------------------------------------------
    O_T_EE_d.setZero();

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();
    nullspace_stiffness_.setZero();

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    R_d_.setZero();

    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    velocity_d_.setZero();

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

    EE_force.setZero();
    EE_force_last.setZero();
    EE_force_filtered.setZero();
    
    count = 0;

    return true;
}


void PolishingController::starting(const ros::Time& /*time*/) {

    q_start << 0, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4;
    FK(forward_kinematics, q_start);
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            O_T_EE_d(i,j) = forward_kinematics(i,j);
        }
    }
    Eigen::Affine3d initial_transform(O_T_EE_d);
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


void PolishingController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

    // ---------------------------------------------------------------------------
    // get state variables
    // ---------------------------------------------------------------------------
    for (int i = 0; i < 7; ++i) {
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
    Eigen::Quaterniond orientation(transform.linear());

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
    // EE frame 
    Eigen::Matrix3d Kp(R_d_ * Kp_d_ * R_d_.transpose());  // cartesian position stiffness
    Eigen::Matrix3d Dp(R_d_ * Dp_d_ * R_d_.transpose());  // cartesian position damping
    Eigen::Matrix3d Ko(R_d_ * Ko_d_ * R_d_.transpose());  // cartesian orientation stiffness
    Eigen::Matrix3d Do(R_d_ * Do_d_ * R_d_.transpose());  // cartesian orientation damping
    Eigen::Matrix3d Ip(R_d_ * Ip_d_ * R_d_.transpose());  // cartesian position integrative
    Eigen::Matrix3d Io(R_d_ * Io_d_ * R_d_.transpose());  // cartesian position integrative
    
    cartesian_stiffness_.setIdentity();
    cartesian_stiffness_.topLeftCorner(3, 3) << Kp;
    cartesian_stiffness_.bottomRightCorner(3, 3) << Ko;
    //std::cout << "\n" << cartesian_stiffness_ << std::endl;

    cartesian_damping_.setIdentity();
    cartesian_damping_.topLeftCorner(3, 3) << Dp;
    cartesian_damping_.bottomRightCorner(3, 3) << Do;
    //std::cout << "\n" << cartesian_damping_ << std::endl;

    cartesian_integral_.setIdentity();
    cartesian_integral_.topLeftCorner(3, 3) << Ip;
    cartesian_integral_.bottomRightCorner(3, 3) << Io;
    // std::cout << "\n" << cartesian_integral_ << std::endl;


    // ---------------------------------------------------------------------------
    // compute error
    // ---------------------------------------------------------------------------
    // position error
    error.head(3) << position_d_ - position;

    // orientation error
    Eigen::Matrix3d Rcd(R_d_ * R.inverse()); // described in the base frame
    error.tail(3) << R2r(Rcd);

    // velocity error
    Eigen::Matrix<double, 6, 1> velocity(J * dq);
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
    Eigen::VectorXd tau_task(7), tau_d(7), tau_nullspace(7);

    // compute the inertia matrix of the task space
    Eigen::Matrix<double, 6, 6> lambda( (J * M.inverse() * J.transpose()).inverse() );

    // Cartesian PD control 
    // tau_task << J.transpose() * lambda * ( cartesian_damping_ * velocity_error + cartesian_stiffness_ * error );
    tau_task << J.transpose() * ( cartesian_damping_ * velocity_error + cartesian_stiffness_ * error + cartesian_integral_ * integral_error );

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
    // Compute the EE external wrench (force,torque) acting on EE frame
    // ---------------------------------------------------------------------------
    Eigen::Matrix<double, 7, 1> tau_ext = externalTorque(effort, tau_d);
    Eigen::MatrixXd JT_pinv = pseudoInverse(J.transpose(), true); // kinematic pseuoinverse   
    Eigen::Matrix<double, 6, 1> EE_wrench = (JT_pinv * tau_ext); // end-effector wrench (force,torque)
    EE_force << EE_wrench[0], EE_wrench[1], EE_wrench[2];  
    if(count < 2000){
        EE_force.setZero();
    }
    // filtering the EE_force
    for (int i = 0; i < 3; ++i) {
        EE_force_filtered(i) = lowpassFilter(0.001, EE_force[i], EE_force_last[i], 100.0);
    }

    // ---------------------------------------------------------------------------
    // update parameters
    // ---------------------------------------------------------------------------
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

    // publish panda_EE pose
    posePublisherCallback(poseEE_pub, position, orientation);
    
    // publish mold frame
    Eigen::Matrix3d Rmold(points2Rotation(P1, P2, P4));
    Eigen::Vector3d mold_position(P1);
    Eigen::Quaterniond mold_orientation(Rmold);
    publishFrame(br_mold, tf_mold, mold_position, mold_orientation, "/panda_link0", "/polishing_mold");

    // update last integral error
    last_integral_error = integral_error;
    
    // update last EEforce
    EE_force_last = EE_force;
 

    // ---------------------------------------------------------------------------
    // Write to file
    // ---------------------------------------------------------------------------
    Eigen::Vector3d euler_angles(R.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)
    Eigen::Vector3d euler_angles_d_(R_d_.eulerAngles(2, 1, 0)); // RPY->ZYX(2,1,0)
    
    count++;
    double TIME = count/1000.0;
    file_tracking << TIME << " "
                  << position[0] << " " << position_d_[0] << " "
                  << position[1] << " " << position_d_[1] << " "
                  << position[2] << " " << position_d_[2] << " "
                  << wrapTo2PI(euler_angles[0]) << " " << wrapTo2PI(euler_angles_d_[0]) << " "
                  << wrapTo2PI(euler_angles[1]) << " " << wrapTo2PI(euler_angles_d_[1]) << " "
                  << wrapToPI(euler_angles[2]) << " " << wrapToPI(euler_angles_d_[2]) << " "
                  << error[0] << " " << error[1] << " " << error[2] << " "
                  << error[3] << " " << error[4] << " " << error[5] << " "
                  << integral_error[0] << " " << integral_error[1] << " " << integral_error[2] << " "
                  << integral_error[3] << " " << integral_error[4] << " " << integral_error[5] << " "
                  << EE_force[0] << " " << EE_force[1] << " " << EE_force[2] << " "
                  << EE_force_filtered[0] << " " << EE_force_filtered[1] << " " << EE_force_filtered[2] << "\n";

}


Eigen::Vector3d PolishingController::R2r(Eigen::Matrix3d& Rotation){
    Eigen::Vector3d rotation_vector, aux;
    aux << Rotation(2,1) - Rotation(1,2),
           Rotation(0,2) - Rotation(2,0),
           Rotation(1,0) - Rotation(0,1);
    rotation_vector = 0.5 * aux;

    return rotation_vector;
}


double PolishingController::wrapToPI(double& angle){
    double new_angle = atan2(sin(angle), cos(angle));
    return new_angle;
}


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


void PolishingController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    
    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    
    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
    if(last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0){
        orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }

}


void PolishingController::complianceParamCallback(franka_simulation::compliance_paramConfig& config, uint32_t /*level*/){

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

}


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


bool PolishingController::FK(KDL::Frame& kdl_frame, Eigen::Matrix<double, 7, 1>& q_values){
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


bool PolishingController::jacobian(KDL::Jacobian &kdl_jacobian, Eigen::Matrix<double, 7, 1> q_values){
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


bool PolishingController::dynamic(KDL::JntSpaceInertiaMatrix& kdl_inertia, KDL::JntArray& kdl_coriolis, KDL::JntArray& kdl_gravity, Eigen::Matrix<double, 7, 1>& q_values, Eigen::Matrix<double, 7, 1>& dq_values, Vector& g_vector){
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


double PolishingController::derivative_computation( const double q_i, const double maxJointLimit_i, const double minJointLimit_i){
    double result;
    double average_joint;
    average_joint = ( maxJointLimit_i + minJointLimit_i) / 2.0;
    result = - ( ( ( q_i - average_joint ) / pow( ( maxJointLimit_i - minJointLimit_i ), 2 ) ) );

    return result;
}


template<int N> // number of joints or DOF
void PolishingController::gradient_mechanical_joint_limit( Eigen::Matrix<double, N, 1>& gradient_mechanical_joint_limit_out, const Eigen::Matrix<double, N, 1> q, const Eigen::Matrix<double, N, 1> maxJointLimits, const Eigen::Matrix<double, N, 1> minJointLimits ){
    for ( int i = 0; i < q.rows(); i++ ){
        gradient_mechanical_joint_limit_out(i) = derivative_computation( q(i), maxJointLimits(i), minJointLimits(i) );
    }
}


void PolishingController::publishFrame(tf::TransformBroadcaster& br, tf::Transform& transform, Eigen::Vector3d& position, Eigen::Quaterniond& orientation, std::string base_link, std::string link_name){
    transform.setOrigin( tf::Vector3(position(0), position(1), position(2)) );
    transform.setRotation( tf::Quaternion(orientation.vec()[0], orientation.vec()[1], orientation.vec()[2], orientation.w()) );
    br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), base_link, link_name) );
}


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


Eigen::Matrix<double, 7, 1> PolishingController::externalTorque(Eigen::Matrix<double, 7, 1>& effort, Eigen::VectorXd& command_torque){
    Eigen::Matrix<double, 7, 1> tau_ext = effort - command_torque;
    return tau_ext;
}


Eigen::MatrixXd PolishingController::pseudoInverse(const Eigen::MatrixXd& M_, bool damped){
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    Eigen::MatrixXd M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());

    return M_pinv_;
}


double PolishingController::lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency) {  
    double gain = sample_time / (sample_time + (1.0 / (2.0 * M_PI * cutoff_frequency)));   
    return gain * y + (1 - gain) * y_last;
}


} // namespace franka_simulation

PLUGINLIB_EXPORT_CLASS(franka_simulation::PolishingController, controller_interface::ControllerBase)
