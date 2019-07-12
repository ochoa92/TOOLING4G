#include <franka_udrilling/spacenav.h>

namespace franka_udrilling{

Spacenav::Spacenav(ros::NodeHandle &nh): nh_(nh) {
  ros::Rate loop_rate(1000);

  spacenav_motion.setZero();

  position_d.setZero();
  orientation_d.coeffs() << 0.0, 0.0, 0.0, 1.0;

  T_spacenav.setZero();
  Tx.setZero();
  Ty.setZero();
  Tz.setZero();

  createPublishersAndSubscribers();

  nh_.getParam("link_name", link_name);

  while(ros::ok() && !franka_state_flag) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

Spacenav::~Spacenav() {
  pose_pub.shutdown();
}


void Spacenav::createPublishersAndSubscribers(){
  // Create subscribers
  spacenav_sub = nh_.subscribe("/spacenav/joy", 20, &Spacenav::joyCallback, this);
  franka_state_sub = nh_.subscribe("/franka_state_controller/franka_states", 20, &Spacenav::frankaStateCallback, this);

  // Create publishers
  pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/panda_equilibrium_pose", 20);
}


void Spacenav::frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg){

  O_T_EE = Eigen::Matrix4d::Map(msg->O_T_EE.data());
  O_F_ext_hat_K = Eigen::Matrix<double, 6, 1>::Map(msg->O_F_ext_hat_K.data());
  K_F_ext_hat_K = Eigen::Matrix<double, 6, 1>::Map(msg->K_F_ext_hat_K.data());
  franka_state_flag = 1;
}


void Spacenav::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {

  double threshold = 0.5, K = 0;
  ros::Duration d(0.5);

  for (int i = 0; i < 6; i++) {
    if (msg->axes[i] >= threshold || msg->axes[i] <= -threshold) {
      if (i >= 0 && i <= 2)
        K = 0.0002;
      else if (i >= 3 && i <= 5)
        K = 0.0005;

      spacenav_motion(i) = K * msg->axes[i];
    }

    else if (msg->axes[i] <= threshold || msg->axes[i] >= -threshold)
      spacenav_motion(i) = 0;
  }

  time_lapse = ros::Time::now();
  if (msg->buttons[0] && time_lapse - last_time > d) {
    last_time = time_lapse;
    if (flag_mode == 0)
      flag_mode = 1;
    else if (flag_mode == 1)
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
      std::cout << CLEANWINDOW << "uDRILLING MODE..." << std::endl;

      Eigen::Matrix4d T_aux;
      T_aux << 1, 0, 0,               0,
               0, 1, 0,               0,
               0, 0, 1, -T_spacenav(2,3),
               0, 0, 0,               1;

      Xd = last_Xd * T_aux;
      aff_d.matrix() = Xd;
      position_d = aff_d.translation();
      orientation_d = aff_d.linear();
      orientation_d.normalize();

      break;

  }

}


void Spacenav::posePublisherCallback(geometry_msgs::PoseStamped& marker_pose, Eigen::Vector3d& position, Eigen::Quaterniond& quaternion){

  marker_pose.pose.position.x = position[0];
  marker_pose.pose.position.y = position[1];
  marker_pose.pose.position.z = position[2];

  marker_pose.pose.orientation.x = quaternion.vec()[0];
  marker_pose.pose.orientation.y = quaternion.vec()[1];
  marker_pose.pose.orientation.z = quaternion.vec()[2];
  marker_pose.pose.orientation.w = quaternion.w();

  // run pose publisher
  marker_pose.header.frame_id = link_name;
  marker_pose.header.stamp = ros::Time::now();
  pose_pub.publish(marker_pose);

}


Eigen::Vector3d Spacenav::polynomial3_trajectory(Eigen::Vector3d& pi, Eigen::Vector3d& pf, double ti, double tf, double t){
  Eigen::Vector3d pd = pi + (3*(pf - pi)*pow((t - ti), 2))/pow((tf - ti), 2) - (2*(pf - pi)*pow((t - ti), 3))/pow((tf - ti), 3);
  return pd;
}


Eigen::VectorXd Spacenav::robot_pose(Eigen::Matrix4d& Xd){

  Eigen::Vector3d position;
  Eigen::Quaterniond quaternion;
  Eigen::Affine3d aff;
  Eigen::VectorXd pose(7,1);

  aff.matrix() = Xd;
  position = aff.translation();
  quaternion = aff.linear();
  quaternion.normalize();

  pose << position[0],
          position[1],
          position[2],
          quaternion.vec()[0],
          quaternion.vec()[1],
          quaternion.vec()[2],
          quaternion.w();

  return pose;
}


} // namespace franka_udrilling
