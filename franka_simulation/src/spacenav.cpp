#include <franka_simulation/spacenav.h>

namespace franka_simulation{

Spacenav::Spacenav(ros::NodeHandle &nh): nh_(nh) {
    ros::Rate loop_rate(1000);

    spacenav_motion.setZero();

    position_d.setZero();
    orientation_d.coeffs() << 0.0, 0.0, 0.0, 1.0;

    T_spacenav.setZero();
    Tx.setZero();
    Ty.setZero();
    Tz.setZero();

    // Create subscribers
    spacenav_sub = nh_.subscribe("/spacenav/joy", 20, &Spacenav::joyCallback, this);
    franka_state_sub = nh_.subscribe("/panda_poseEE", 20, &Spacenav::pandaStateCallback, this);

    // Create publishers
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/panda_equilibrium_pose", 20);
    mf_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/panda_hand_joint_trajectory_controller/command", 20);

    while(ros::ok() && !panda_state_flag) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

Spacenav::~Spacenav() {
    pose_pub.shutdown();
}


void Spacenav::pandaStateCallback(const geometry_msgs::PoseStampedConstPtr& msg){

    Eigen::Affine3d aff;
    aff.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    
    Eigen::Quaterniond orientation;
    orientation.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;  
    Eigen::Matrix3d R(orientation.toRotationMatrix());
    aff.linear() << R;
    
    O_T_EE = aff.matrix();
    
    panda_state_flag = 1;
}


void Spacenav::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {

    double threshold = 0.5, K = 0;
    ros::Duration d(0.5);

    for (int i = 0; i < 6; i++) {
        if (msg->axes[i] >= threshold || msg->axes[i] <= -threshold) {
            if (i >= 0 && i <= 2){
                K = 0.0005;
            }              
            else if (i >= 3 && i <= 5){
                K = 0.0005;
            }              
            spacenav_motion(i) = K * msg->axes[i];
        }

        else if (msg->axes[i] <= threshold || msg->axes[i] >= -threshold){
            spacenav_motion(i) = 0;
        }
        
    }

    time_lapse = ros::Time::now();
    if (msg->buttons[0] && time_lapse - last_time > d){
        last_time = time_lapse;
    if (flag_mode == 0)
        flag_mode = 1;
    else if (flag_mode == 1)
        flag_mode = 2;
    else if (flag_mode == 2)
        flag_mode = 3;
    else if (flag_mode == 3)
        flag_mode = 0;
    }

    spacenav_button_1 = msg->buttons[0];
    spacenav_button_2 = msg->buttons[1];
}


void Spacenav::MotionControl(Eigen::Matrix4d& Xd){

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

    Eigen::Vector2d fingers_position;

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

        Ty << cos(spacenav_motion(4)), 0, sin(spacenav_motion(4)),                  0,
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
            std::cout << CLEANWINDOW << "OPERATING ROTATION IN END-EFFECTOR AND TRANSLATION IN BASE..." << std::endl;

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
            position_d = aff_d.translation();
            orientation_d = aff_d.linear();
            orientation_d.normalize();

            break;

        case 1:
            std::cout << CLEANWINDOW << "OPERATING IN BASE...\n\n";

            Xd = T_spacenav * last_Xd;

            aff_d.matrix() = Xd;
            position_d = aff_d.translation();
            orientation_d = aff_d.linear();
            orientation_d.normalize();

            break;

        case 2:
            std::cout << CLEANWINDOW << "OPERATING IN END-EFFECTOR...\n\n";

            Xd = last_Xd * T_spacenav;

            aff_d.matrix() = Xd;
            position_d = aff_d.translation();
            orientation_d = aff_d.linear();
            orientation_d.normalize();

            break;

        case 3:
            std::cout << CLEANWINDOW << "PANDA FINGERS MODE...\n\n";

            if(spacenav_motion(1) > 0.0){
                fingers_position << 0.0, 0.0;     
                moveFingersCallback(fingers_position);   
            }
            else if(spacenav_motion(1) < 0.0){
                fingers_position << 1.0, 1.0;
                moveFingersCallback(fingers_position);   
            }
            
            break;

    }

}


void Spacenav::posePublisherCallback(Eigen::Vector3d& position, Eigen::Quaterniond& orientation){

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


Eigen::Vector3d Spacenav::polynomial3_trajectory(Eigen::Vector3d& pi, Eigen::Vector3d& pf, double ti, double tf, double t){
    Eigen::Vector3d pd = pi + (3*(pf - pi)*pow((t - ti), 2))/pow((tf - ti), 2) - (2*(pf - pi)*pow((t - ti), 3))/pow((tf - ti), 3);
    return pd;
}


Eigen::VectorXd Spacenav::robotPose(Eigen::Matrix4d& Xd){

    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Affine3d aff;
    Eigen::VectorXd pose(7,1);

    aff.matrix() = Xd;
    position = aff.translation();
    orientation = aff.linear();
    orientation.normalize();

    pose << position[0],
            position[1],
            position[2],
            orientation.vec()[0],
            orientation.vec()[1],
            orientation.vec()[2],
            orientation.w();

    return pose;
}


void Spacenav::moveFingersCallback(Eigen::Vector2d& position){
    
    trajectory_msgs::JointTrajectory fingersCmd;

    fingersCmd.header.stamp = ros::Time::now();
    fingersCmd.header.frame_id = "/panda_link0";

    fingersCmd.joint_names.resize(2);
    fingersCmd.joint_names[0] = "panda_finger_joint1";
    fingersCmd.joint_names[1] = "panda_finger_joint2";

    fingersCmd.points.resize(1);
    fingersCmd.points[0].positions.resize(2);
    fingersCmd.points[0].velocities.resize(2);
    fingersCmd.points[0].accelerations.resize(2);
    fingersCmd.points[0].effort.resize(2);

    fingersCmd.header.stamp = ros::Time::now() + ros::Duration(0.0);
    fingersCmd.points[0].time_from_start = ros::Duration(0.5);
    for(int i=0; i<2; i++){
        fingersCmd.points[0].positions[i] = position(i);
        fingersCmd.points[0].velocities[i] = 0;
        fingersCmd.points[0].accelerations[i] = 0;
        fingersCmd.points[0].effort[i] = 0;
    }
    mf_pub.publish(fingersCmd);

}


} // namespace franka_simulation
