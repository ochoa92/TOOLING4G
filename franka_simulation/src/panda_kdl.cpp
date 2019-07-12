#include <franka_simulation/panda_kdl.h>

using namespace P_KDL;

panda_kdl::panda_kdl(ros::NodeHandle &nh): nh_(nh){


  ros::Rate loop_rate(1000);

  // create subscribers
  joint_states_sub = nh.subscribe("/joint_states", 1, &panda_kdl::joint_state_callback, this);

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
  num_joints = kdl_chain.getNrOfJoints();

  // Initializations
  joint_position.resize(num_joints);
  joint_velocity.resize(num_joints);
  joint_effort.resize(num_joints);


  // Wait for first message from joint_state subscriber arrives
  while(ros::ok() && !joint_state_flag){
    ros::spinOnce();
    loop_rate.sleep();
  }
}

panda_kdl::~panda_kdl(){}

void panda_kdl::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg){
  for(int i=0; i<num_joints; i++){
    joint_position(i) = (double)(msg->position[i]);
    joint_velocity(i) = (double)(msg->velocity[i]);
    joint_effort(i) = (double)(msg->effort[i]);
  }
  joint_state_flag = 1;
}

bool panda_kdl::FK(KDL::Frame &kdl_frame){
  return FK(kdl_frame, joint_position);
}

bool panda_kdl::FK(KDL::Frame &kdl_frame, Eigen::Matrix<double, 7, 1> q_values){
  if (q_values.size() != num_joints){
    return false;
  }

  // Create KDL FK Solver
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(kdl_chain);

  // Create Joint Array for calculations
  KDL::JntArray joint_position_array = JntArray(num_joints);

  for (int i = 0; i < num_joints; i++){
    joint_position_array(i) = q_values[i];
  }

  fksolver.JntToCart(joint_position_array, kdl_frame);

  return true;
}

bool panda_kdl::jacobian(KDL::Jacobian &kdl_jacobian){
  return jacobian(kdl_jacobian, joint_position);
}

bool panda_kdl::jacobian(KDL::Jacobian &kdl_jacobian, Eigen::Matrix<double, 7, 1> q_values){
  if(q_values.size() != num_joints){
    return false;
  }
  else{
    // create KDL Jacobian Solver
    ChainJntToJacSolver jac_solver = ChainJntToJacSolver(kdl_chain);

    // create Joint Array for calculations
    KDL::JntArray joint_position_array = JntArray(num_joints);

    // define Jacobian with the correct number of columns
    kdl_jacobian = Jacobian(num_joints);

    for(int i=0; i<num_joints; i++){
      joint_position_array(i) = q_values[i];
    }

    jac_solver.JntToJac(joint_position_array, kdl_jacobian);

    return true;
  }
}

bool panda_kdl::dynamic(KDL::JntSpaceInertiaMatrix &kdl_inertia, KDL::JntArray &kdl_coriolis, KDL::JntArray &kdl_gravity, Vector gravity){
  return dynamic(kdl_inertia, kdl_coriolis, kdl_gravity, joint_position, joint_velocity, gravity);
}

bool panda_kdl::dynamic(KDL::JntSpaceInertiaMatrix &kdl_inertia, KDL::JntArray &kdl_coriolis, KDL::JntArray &kdl_gravity, Eigen::Matrix<double, 7, 1> q_values, Eigen::Matrix<double, 7, 1> dq_values, Vector gravity){
  if(q_values.size() != num_joints || dq_values.size() != num_joints){
    return false;
  }
  else{
    // create KDL Dynamic Solver
    ChainDynParam dyn_solver = ChainDynParam(kdl_chain, gravity);

    // create Joint Array for calculations
    KDL::JntArray joint_position_array = JntArray(num_joints);
    KDL::JntArray joint_velocity_array = JntArray(num_joints);

    // define inertia matrix with correct size (rows and columns)
    kdl_inertia = JntSpaceInertiaMatrix(num_joints);
    kdl_coriolis = JntArray(num_joints);
    kdl_gravity = JntArray(num_joints);

    for(int i=0; i<num_joints; i++){
      joint_position_array(i) = q_values[i];
      joint_velocity_array(i) = dq_values[i];
    }

    dyn_solver.JntToMass(joint_position_array, kdl_inertia);
    dyn_solver.JntToCoriolis(joint_position_array, joint_velocity_array, kdl_coriolis);
    dyn_solver.JntToGravity(joint_position_array, kdl_gravity);

    return true;
  }
}
