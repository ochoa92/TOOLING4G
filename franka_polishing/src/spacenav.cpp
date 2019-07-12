#include <franka_polishing/spacenav.h>

namespace franka_polishing{

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
      position_d = aff_d.translation();
      orientation_d = aff_d.linear();
      orientation_d.normalize();

      break;

    case 1:
      if(flag_print == 1){
        std::cout << CLEANWINDOW << "POLISHING MODE..." << std::endl;
        flag_print = 0;
      }

      // rotation in EE frame
      last_aff_d.matrix() = last_Xd;
      last_R_d = last_aff_d.rotation();

      aff_spacenav.matrix() = T_spacenav;
      R_spacenav.setIdentity();

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


Eigen::Vector3d Spacenav::polynomial3_trajectory(Eigen::Vector3d& pi, Eigen::Vector3d& pf, double& ti, double& tf, double& t){
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


Eigen::Matrix3d Spacenav::points2Rotation(Eigen::Vector3d& P1, Eigen::Vector3d& P2, Eigen::Vector3d& P3){

  // compute the vectors of the desired rotation matrix
  Eigen::Vector3d nx( (P3-P1)/((P3-P1).norm()) );
  Eigen::Vector3d ny( (P2-P1)/((P2-P1).norm()) );
  Eigen::Vector3d nz( nx.cross(ny) );
  nz = nz/(nz.norm());

  // complete the rotation matrix (orthogonality characteristic)
  ny = nz.cross(nx);
  ny = ny/(ny.norm());

  // construct the rotation matrix R = [nx', ny', nz']
  Eigen::Matrix3d Rmould;
  Rmould << nx(0), ny(0), nz(0),
            nx(1), ny(1), nz(1),
            nx(2), ny(2), nz(2);

  return Rmould;
}


visualization_msgs::Marker Spacenav::pointsMarker(std::string points_ns, int points_id, Eigen::Vector3d points_scale, Eigen::Vector3d points_color){

  visualization_msgs::Marker points;
  points.header.frame_id = "/panda_link0";
  points.header.stamp = ros::Time::now();
  points.ns = points_ns;
  points.id = points_id;
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::POINTS;
  points.pose.orientation.w = 1.0;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = points_scale(0);
  points.scale.y = points_scale(1);
  points.scale.z = points_scale(2);

  // Set the color -- be sure to set alpha to something non-zero!
  points.color.a = 1.0;
  points.color.r = points_color(0);
  points.color.g = points_color(1);
  points.color.b = points_color(2);

  return points;
}


visualization_msgs::Marker Spacenav::lineStripsMarker(std::string lines_ns, int lines_id, double lines_scale, Eigen::Vector3d lines_color){
  visualization_msgs::Marker line_strips;
  line_strips.header.frame_id = "/panda_link0";
  line_strips.header.stamp = ros::Time::now();
  line_strips.ns = lines_ns;
  line_strips.id = lines_id;
  line_strips.action = visualization_msgs::Marker::ADD;
  line_strips.type = visualization_msgs::Marker::LINE_STRIP;
  line_strips.pose.orientation.w = 1.0;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strips.scale.x = lines_scale;

  // Set the color -- be sure to set alpha to something non-zero!
  line_strips.color.a = 1.0;
  line_strips.color.r = lines_color(0);
  line_strips.color.g = lines_color(1);
  line_strips.color.b = lines_color(2);

  return line_strips;
}


Eigen::MatrixXd Spacenav::moldWorkSpace(Eigen::Vector3d& P1, Eigen::Vector3d& P2, Eigen::Vector3d& P3, Eigen::Vector3d& delta_synthetic, int& Nx, int& Ny){

  double step_x = 1/(double)(Nx-1);
  double step_y = 1/(double)(Ny-1);

  Eigen::Vector3d P1_synthetic(P1 + delta_synthetic);
  Eigen::Vector3d P2_synthetic(P2 + delta_synthetic);
  Eigen::Vector3d P3_synthetic(P3 + delta_synthetic);
  Eigen::Vector3d vec_x(P3_synthetic - P1_synthetic);
  Eigen::Vector3d vec_y(P2_synthetic - P1_synthetic);

  Eigen::Vector3d WS_initial(P1 + vec_x/(2*Nx) + vec_y/(2*Ny));

  Eigen::MatrixXd WS;
  WS.resize(3, (Nx-1)*(Ny-1));
  int n = 0;  // number of columns
  for(int i = 0; i < Nx-1; ++i){
    for(int j = 0; j < Ny-1; ++j){
      WS(0, n) = WS_initial(0) + (vec_x(0)*step_x*i + vec_y(0)*step_y*j);
      WS(1, n) = WS_initial(1) + (vec_x(1)*step_x*i + vec_y(1)*step_y*j);
      WS(2, n) = WS_initial(2) + (vec_x(2)*step_x*i + vec_y(2)*step_y*j);
      ++n;
    }
  }

  return WS;
}


} // namespace franka_polishing
