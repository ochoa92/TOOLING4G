// =============================================================================
// Name        : z_capsule_node.cpp
// Author      : Hélio Ochoa
// Description :
// =============================================================================
#include <franka_spacenav/spacenav.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "z_capsule_node");

  ros::NodeHandle nh;
  franka_spacenav::Spacenav franka(nh);

  geometry_msgs::PoseStamped marker_pose;

  // ---------------------------------------------------------------------------
  // GET POINTS FROM A FILE
  // ---------------------------------------------------------------------------
  MatrixXd POINTS;  // matrix to save the mould points
  ifstream file_points;
  string path_points;
  path_points = "/home/panda/catkin_ws/src/TOOLING4G/franka_spacenav/spacenav_data/points";
  file_points.open(path_points);

  double X, Y, Z;
  int points_column = 0;
  POINTS.resize(3, points_column + 1);
  if(file_points.is_open()){
    while(file_points >> X >> Y >> Z){
      // save the values in the matrix
      POINTS.conservativeResize(3, points_column + 1);
      POINTS(0, points_column) = X;
      POINTS(1, points_column) = Y;
      POINTS(2, points_column) = Z;
      points_column++;
    }
  }
  else{
    cout << "\nError open the file!" << endl;
    return(0);
  }
  file_points.close();
  // std::cout << POINTS.transpose() << std::endl;

  // Markers
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20);

  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = "/panda_link0";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = "points";
  line_strip.ns = "lines";
  points.id = 0;
  line_strip.id = 1;
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.01;
  points.scale.y = 0.01;
  points.scale.z = 0.01;

  // Set the color -- be sure to set alpha to something non-zero!
  points.color.a = 1.0;
  points.color.r = 1.0;
  points.color.g = 0.0;
  points.color.b = 0.0;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.01;

  // Line strip is green
  line_strip.color.a = 1.0;
  line_strip.color.g = 1.0;

  geometry_msgs::Point p, l;  // points and lines
  int i = 0;
  while(i < points_column){
    p.x = POINTS(0, i);
    p.y = POINTS(1, i);
    p.z = POINTS(2, i);
    points.points.push_back(p);
    i++;
  }


  // ---------------------------------------------------------------------------
  // GET THE DESIRED ORIENTATION
  // ---------------------------------------------------------------------------
  ifstream file_desired_o;
  string path_desired_o;
  path_desired_o = "/home/panda/catkin_ws/src/TOOLING4G/franka_spacenav/spacenav_data/desired_o";
  file_desired_o.open(path_desired_o);

  double qx, qy, qz, qw;
  if(file_desired_o.is_open()){
    file_desired_o >> qx >> qy >> qz >> qw;
  }
  else{
    std::cout << "Error open the file!" << std::endl;
    return(0);
  }
  file_desired_o.close();
  Quaterniond Qd;  // desired Quaternion
  Qd.vec()[0] = qx;
  Qd.vec()[1] = qy;
  Qd.vec()[2] = qz;
  Qd.w() = qw;
  // std::cout << Qd.coeffs() << std::endl;

  Matrix3d Rd(Qd); // desired Rotation
  // std::cout << Rd << std::endl;


  // ---------------------------------------------------------------------------
  // tf broadcaster
  // ---------------------------------------------------------------------------
  tf::TransformBroadcaster br_d;
  tf::Transform transform_d;


  // ---------------------------------------------------------------------------
  // GET INITIAL POSE
  // ---------------------------------------------------------------------------
  Matrix4d O_T_EE_i;
  VectorXd pose_i(7,1);
  O_T_EE_i = franka.FK;
  pose_i = franka.robot_pose(O_T_EE_i);

  // ---------------------------------------------------------------------------
  // TRAJECTORY TO FIRST POINT
  // ---------------------------------------------------------------------------
  // position
  Vector3d pi, pf;
  pi << pose_i[0], pose_i[1], pose_i[2];
  pf << POINTS(0, 0), POINTS(1, 0), POINTS(2, 0);
  double ti = 2.0;
  double tf = 5.0;
  double t = 0.0;
  double delta_t = 0.001;

  // orientation
  Quaterniond oi, of;
  oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
  of.coeffs() << Qd.vec()[0], Qd.vec()[1], Qd.vec()[2], Qd.w();
  double t1 = 0.0;
  double delta_t1 = delta_t/(tf-ti);


  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  Vector3d position_d(pi);
  Quaterniond orientation_d(oi);

  int flag_point = 0;
  int flag_print = 0;

  ros::Rate loop_rate(1000);
  while (ros::ok()){

    // -------------------------------------------------------------------------
    // INITIAL POSE -> P1
    // -------------------------------------------------------------------------
    if(flag_point == 0){
      if(flag_print == 0){
        cout << CLEANWINDOW << "ROBOT IS MOVING TO THE FIRST POINT..." << endl;
        flag_print = 1;
      }

      if( (t >= ti) && (t <= tf) ){
        position_d = franka.polynomial3_traj3D(pi, pf, ti, tf, t);
        if ( t1 <= 1.0 ){
          orientation_d = oi.slerp(t1, of);
          orientation_d.normalize();
        }
        t1 = t1 + delta_t1;
      }
      else if(t > tf){
        if(flag_print == 1){
          cout << CLEANWINDOW << "PLEASE PRESS SPACENAV BUTTON <1> TO START THE TRAJECTORY!" << endl;
          flag_print = 2;
        }
        if(franka.spacenav_button_1 == 1){
          flag_point = 1;
          O_T_EE_i = franka.FK;
          pose_i = franka.robot_pose(O_T_EE_i);
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << POINTS(0, 1), POINTS(1, 1), POINTS(2, 1);
          t = 0;  // reset time
        }
      }
      t = t + delta_t;

    }

    // -------------------------------------------------------------------------
    // P1 -> P2
    // -------------------------------------------------------------------------
    if(flag_point == 1){
      if(flag_print == 2){
        cout << CLEANWINDOW << "ROBOT IS MOVING TO THE SECOND POINT..." << endl;
        flag_print = 3;
      }

      ti = 0.0;
      tf = 10.0;
      if( (t >= ti) && (t <= tf) ){
        position_d = franka.polynomial3_traj3D(pi, pf, ti, tf, t);
      }
      else if(t > tf){
        flag_point = 2;
        O_T_EE_i = franka.FK;
        pose_i = franka.robot_pose(O_T_EE_i);
        oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
        Matrix3d Ri(oi); // initial Rotation
        double angle = 3*PI/4;
        int axis = 2;
        Matrix3d Rz(franka.rotate(angle, axis));
        Matrix3d Rf(Ri*Rz); // final Rotation
        of = Rf;
        t1 = 0;  // reset time
      }
      t = t + delta_t;

    }

    // -------------------------------------------------------------------------
    // ROTATE 1
    // -------------------------------------------------------------------------
    if(flag_point == 2){
      if(flag_print == 3){
        cout << CLEANWINDOW << "ROBOT IS CHANGING ITS ORIENTATION..." << endl;
        flag_print = 4;
      }

      // --> MOVE DOWN <--
      ti = 0.0;
      tf = 10.0;
      delta_t1 = delta_t/(tf-ti);
      if ( t1 <= 1.0 ){
        orientation_d = oi.slerp(t1, of);
        orientation_d.normalize();
      }
      else if(t1 > 1.0){
        flag_point = 3;
        O_T_EE_i = franka.FK;
        pose_i = franka.robot_pose(O_T_EE_i);
        pi << pose_i[0], pose_i[1], pose_i[2];
        pf << POINTS(0, 2), POINTS(1, 2), POINTS(2, 2);
        t = 0;  // reset time
      }
      t1 = t1 + delta_t1;


    }

    // -------------------------------------------------------------------------
    // P2 -> P3
    // -------------------------------------------------------------------------
    if(flag_point == 3){
      if(flag_print == 4){
        cout << CLEANWINDOW << "ROBOT IS MOVING TO THE THIRD POINT..." << endl;
        flag_print = 5;
      }

      ti = 0.0;
      tf = 10.0;
      if( (t >= ti) && (t <= tf) ){
        position_d = franka.polynomial3_traj3D(pi, pf, ti, tf, t);
      }
      else if(t > tf){
        flag_point = 4;
        O_T_EE_i = franka.FK;
        pose_i = franka.robot_pose(O_T_EE_i);
        oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
        Matrix3d Ri(oi); // initial Rotation
        double angle = -3*PI/4;
        int axis = 2;
        Matrix3d Rz(franka.rotate(angle, axis));
        Matrix3d Rf(Ri*Rz); // final Rotation
        of = Rf;
        t1 = 0;  // reset time
      }
      t = t + delta_t;

    }

    // -------------------------------------------------------------------------
    // ROTATE 2
    // -------------------------------------------------------------------------
    if(flag_point == 4){
      if(flag_print == 5){
        cout << CLEANWINDOW << "ROBOT IS CHANGING ITS ORIENTATION..." << endl;
        flag_print = 6;
      }

      // --> MOVE DOWN <--
      ti = 0.0;
      tf = 10.0;
      delta_t1 = delta_t/(tf-ti);
      if ( t1 <= 1.0 ){
        orientation_d = oi.slerp(t1, of);
        orientation_d.normalize();
      }
      else if(t1 > 1.0){
        flag_point = 5;
        O_T_EE_i = franka.FK;
        pose_i = franka.robot_pose(O_T_EE_i);
        pi << pose_i[0], pose_i[1], pose_i[2];
        pf << POINTS(0, 3), POINTS(1, 3), POINTS(2, 3);
        t = 0;  // reset time
      }
      t1 = t1 + delta_t1;
    }


    // -------------------------------------------------------------------------
    // P3 -> P4
    // -------------------------------------------------------------------------
    if(flag_point == 5){
      if(flag_print == 6){
        cout << CLEANWINDOW << "ROBOT IS MOVING TO THE LAST POINT..." << endl;
        flag_print = 7;
      }

      ti = 0.0;
      tf = 10.0;
      if( (t >= ti) && (t <= tf) ){
        position_d = franka.polynomial3_traj3D(pi, pf, ti, tf, t);
      }
      else if(t > tf){
        if(flag_print == 7){
          cout << CLEANWINDOW << "TRAJECTORY COMPLETE!" << endl;
          flag_print = 8;
        }
        flag_point = 6;
      }
      t = t + delta_t;

    }


    // cout << CLEANWINDOW << position_d << endl;
    // cout << CLEANWINDOW << orientation_d.coeffs() << endl;
    franka.publisherCallback(marker_pose, position_d, orientation_d);

    // -------------------------------------------------------------------------
    // TF AND VISUALIZATION MARKERS
    // -------------------------------------------------------------------------
    // Draw the desired EE transform
    transform_d.setOrigin( tf::Vector3(position_d(0), position_d(1), position_d(2)) );
    transform_d.setRotation( tf::Quaternion(orientation_d.vec()[0], orientation_d.vec()[1], orientation_d.vec()[2], orientation_d.w()) );
    br_d.sendTransform(tf::StampedTransform(transform_d, ros::Time::now(), "/panda_link0", "/panda_EE_d"));

    // Draw the points of the mould work space
    marker_pub.publish(points);
    // -------------------------------------------------------------------------

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
