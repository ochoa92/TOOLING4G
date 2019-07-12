  #include <franka_spacenav/spacenav.h>

namespace franka_spacenav{

Spacenav::Spacenav(ros::NodeHandle &nh): nh_(nh){

  ros::Rate loop_rate(1000);

  spacenav_motion.setZero();
  FK.setZero();

  position_d.setZero();
  orientation_d.coeffs() << 0.0, 0.0, 0.0, 1.0;

  T_spacenav.setZero();
  Tx.setZero();
  Ty.setZero();
  Tz.setZero();

  createSubscribersAndPublishers();

  nh_.getParam("link_name", link_name);

  // Wait for first message from joint state subscriber arrives
  while(ros::ok() && !franka_state_flag){
    ros::spinOnce();
    loop_rate.sleep();
  }

}

Spacenav::~Spacenav(){
  pose_pub.shutdown();
}


void Spacenav::createSubscribersAndPublishers(void){
  // Create subscribers
  spacenav_sub = nh_.subscribe("/spacenav/joy", 20, &Spacenav::joy_callback, this);
  franka_state_sub = nh_.subscribe("/franka_state_controller/franka_states", 20, &Spacenav::franka_state_callback, this);

  // Create publishers
  pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/panda_equilibrium_pose", 20);

}


void Spacenav::franka_state_callback(const franka_msgs::FrankaState::ConstPtr &msg){

  int k = 0;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      FK(j,i) = msg->O_T_EE[k];
      k++;
    }
  }

  franka_state_flag = 1;
}


void Spacenav::joy_callback(const sensor_msgs::Joy::ConstPtr &msg){

  double threshold = 0.5, K = 0;
  ros::Duration d(0.5);

  for (int i = 0; i < 6; i++){
    if (msg->axes[i] >= threshold || msg->axes[i] <= -threshold)
    {
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

  spacenav_button_1 = msg->buttons[0];
  if (msg->buttons[0] && time_lapse - last_time_mode > d){
    last_time_mode = time_lapse;
    if (flag_mode == 0)
      flag_mode = 1;
    else if (flag_mode == 1)
      flag_mode = 2;
    else if (flag_mode == 2)
      flag_mode = 3;
    else if (flag_mode == 3)
      flag_mode = 0;
  }

  spacenav_button_2 = msg->buttons[1];
  if (msg->buttons[1] && time_lapse - last_time_gripper > d){
    last_time_gripper = time_lapse;
    gripper_move_client(width, speed);
  }


}


void Spacenav::MotionControl(Matrix4d &Xd){

  Matrix4d last_Xd;

  Matrix3d last_R_d;
  Matrix3d R_spacenav;
  Matrix3d R_EE; // rotation in End-Effector frame
  Vector3d last_position_d;
  Vector3d position_spacenav;
  Vector3d position_B;  // position in base frame

  Affine3d aff_d;
  Affine3d last_aff_d;
  Affine3d aff_spacenav;

  // ---------------------------------------------------------------------------
  // GRIPPER:
  // ---------------------------------------------------------------------------
  double delta_width = 0.00005;
  double delta_speed = 0.0001;
  // ---------------------------------------------------------------------------

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
      cout << CLEANWINDOW << "OPERATING ROTATION IN END-EFFECTOR AND TRANSLATION IN BASE..." << endl;

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
      cout << CLEANWINDOW << "GRIPPER MODE..." << "\n gripper_width(m): " << width << " gripper_speed(m/s): " << speed << endl;

      linear_y = spacenav_motion(1);
      angular_z = spacenav_motion(5);

      // change gripper width
      if(linear_y > 0.0){
        width += delta_width;
        if(width > 0.08){
          width = 0.08;
        }
      }
      else if(linear_y < 0.0){
        width -= delta_width;
        if(width < 0.0){
          width = 0.0;
        }
      }

      // change gripper speed
      if(angular_z > 0.0){
        speed += delta_speed;
        if(speed > 0.1){
          speed = 0.1;
        }
      }
      else if(angular_z < 0.0){
        speed -= delta_speed;
        if(speed < 0.0){
          speed = 0.0;
        }
      }

      break;

    case 2:
      cout << CLEANWINDOW << "OPERATING IN BASE...\n\n";

      Xd = T_spacenav * last_Xd;

      aff_d.matrix() = Xd;
      position_d = aff_d.translation();
      orientation_d = aff_d.linear();
      orientation_d.normalize();

      break;

    case 3:
      cout << CLEANWINDOW << "OPERATING IN END-EFFECTOR...\n\n";

      Xd = last_Xd * T_spacenav;

      aff_d.matrix() = Xd;
      position_d = aff_d.translation();
      orientation_d = aff_d.linear();
      orientation_d.normalize();

      break;

  }

}


void Spacenav::publisherCallback(geometry_msgs::PoseStamped &marker_pose, Vector3d &position, Quaterniond &quaternion){

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


void Spacenav::gripper_move_client(double &width, double &speed){

  // create the action client
  actionlib::SimpleActionClient<franka_gripper::MoveAction> ac("/franka_gripper/move", true);

  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  // send a goal to the action
  franka_gripper::MoveGoal goal;
  goal.width = width;
  goal.speed = speed;
  ac.sendGoal(goal);

}


VectorXd Spacenav::polynomial3_trajectory(const VectorXd &pi, const VectorXd &pf, double ti, double tf, double t){

  VectorXd pd = pi + (3*(pf - pi)*pow((t - ti), 2))/pow((tf - ti), 2) - (2*(pf - pi)*pow((t - ti), 3))/pow((tf - ti), 3);

  return pd;
}


VectorXd Spacenav::robot_pose(Matrix4d &O_T_EE){

  Vector3d position;
  Quaterniond quaternion;
  Affine3d aff;
  VectorXd pose(7,1);

  aff.matrix() = O_T_EE;
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


Vector3d Spacenav::polynomial3_traj3D(Vector3d& pi, Vector3d& pf, double ti, double tf, double t){
  Eigen::Vector3d pd = pi + (3*(pf - pi)*pow((t - ti), 2))/pow((tf - ti), 2) - (2*(pf - pi)*pow((t - ti), 3))/pow((tf - ti), 3);
  return pd;
}


Matrix3d Spacenav::rotate(double angle, int axis){
  // angle (rad)
  // 0: rotation in 'x'
  // 1: rotation in 'y'
  // 3: rotation in 'z'

  Matrix3d R;
  // rotation in 'x'
  if(axis == 0){
    R << 1,          0,           0,
         0, cos(angle), -sin(angle),
         0, sin(angle),  cos(angle);
  }
  // rotation in 'y'
  else if(axis == 1){
    R << cos(angle), 0, sin(angle),
                  0, 1,          0,
        -sin(angle), 0, cos(angle);
  }
  // rotation in 'z'
  else if(axis == 2){
    R << cos(angle), -sin(angle), 0,
         sin(angle),  cos(angle), 0,
                  0,           0, 1;
  }

  return R;
}


} // namespace franka_spacenav
