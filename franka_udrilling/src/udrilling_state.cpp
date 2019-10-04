#include <franka_udrilling/udrilling_state.h>

namespace franka_udrilling{

// ####################################################################################
uDrillingState::uDrillingState(ros::NodeHandle &nh): nh_(nh){
    ros::Rate loop_rate(1000);

    // create publishers -----------------------------------------------
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/panda_equilibrium_pose", 20);

    // create subscribers ----------------------------------------------
    spacenav_sub = nh_.subscribe("/spacenav/joy", 20, &uDrillingState::joyCallback, this);
    franka_state_sub = nh_.subscribe("/franka_state_controller/franka_states", 20, &uDrillingState::frankaStateCallback, this);

    while(ros::ok() && !franka_state_flag){
        ros::spinOnce();
        loop_rate.sleep();
    }

}

// ####################################################################################
uDrillingState::~uDrillingState(){
    pose_pub.shutdown();
}

// ####################################################################################
void uDrillingState::frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg){
    O_T_EE = Eigen::Matrix4d::Map(msg->O_T_EE.data());
    O_F_ext_hat_K = Eigen::Matrix<double, 6, 1>::Map(msg->O_F_ext_hat_K.data());
    K_F_ext_hat_K = Eigen::Matrix<double, 6, 1>::Map(msg->K_F_ext_hat_K.data());
    franka_state_flag = 1;
}

// ####################################################################################
void uDrillingState::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    spacenav_button_1 = msg->buttons[0];
    spacenav_button_2 = msg->buttons[1];
}

// ####################################################################################
Eigen::Vector3d uDrillingState::polynomial3Trajectory(Eigen::Vector3d& pi, Eigen::Vector3d& pf, double ti, double tf, double t){
    Eigen::Vector3d pd = pi + (3*(pf - pi)*pow((t - ti), 2))/pow((tf - ti), 2) - (2*(pf - pi)*pow((t - ti), 3))/pow((tf - ti), 3);
    return pd;
}

// ####################################################################################
Eigen::VectorXd uDrillingState::robotPose(Eigen::Matrix4d& Xd){

    Eigen::Affine3d aff;
    aff.matrix() = Xd;

    Eigen::Vector3d position(aff.translation());
    Eigen::Quaterniond orientation(aff.linear());
    orientation.normalize();

    Eigen::VectorXd pose(7,1);
    pose << position[0], position[1], position[2],
            orientation.vec()[0], orientation.vec()[1], orientation.vec()[2], orientation.w();

    return pose;
}

// ####################################################################################
void uDrillingState::posePublisherCallback(Eigen::Vector3d& position, Eigen::Quaterniond& orientation){

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose.position.x = position[0];
    robot_pose.pose.position.y = position[1];
    robot_pose.pose.position.z = position[2];

    robot_pose.pose.orientation.x = orientation.vec()[0];
    robot_pose.pose.orientation.y = orientation.vec()[1];
    robot_pose.pose.orientation.z = orientation.vec()[2];
    robot_pose.pose.orientation.w = orientation.w();

    robot_pose.header.frame_id = "/panda_link0";
    robot_pose.header.stamp = ros::Time::now();
    pose_pub.publish(robot_pose);

}


} // namespace franka_udrilling
