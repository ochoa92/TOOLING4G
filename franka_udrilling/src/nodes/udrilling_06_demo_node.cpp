// =============================================================================
// Name        : udrilling_06_demo_node.cpp
// Author      : Hélio Ochoa
// Description : 0.6 mm 
// =============================================================================
#include <franka_udrilling/spacenav.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>


// STATE MACHINE ---------------------------------------------------------------
#define MOVE2STATION 0
#define STATIONDOWN 1
#define STATIONUP 2
#define MOVE2POINT 3
#define POINTDOWN 4
#define PREDRILL 5
#define DRILL 6
#define DRILLUP 7
#define DRILLDOWN 8
#define CHANGEPOINT 9
#define INTERRUPT 10
// -----------------------------------------------------------------------------

int main(int argc, char **argv){

  ros::init(argc, argv, "udrilling_06_demo_node");
  ros::NodeHandle nh;
  franka_udrilling::Spacenav panda(nh);

  geometry_msgs::PoseStamped marker_pose;

  // ---------------------------------------------------------------------------
  // VIZUALIZATION MARKERS
  // ---------------------------------------------------------------------------
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20);
  visualization_msgs::Marker points;
  points.header.frame_id = "/panda_link0";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.id = 0;
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::POINTS;
  points.pose.orientation.w = 1.0;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.01;
  points.scale.y = 0.01;
  points.scale.z = 0.01;

  // Set the color -- be sure to set alpha to something non-zero!
  points.color.a = 1.0;
  points.color.r = 1.0;
  points.color.g = 0.0;
  points.color.b = 0.0;


  // ---------------------------------------------------------------------------
  // tf broadcaster
  // ---------------------------------------------------------------------------
  tf::TransformBroadcaster station_br, pandaEEd_br, mould_br;
  tf::Transform station_tf, pandaEEd_tf, mould_tf;


  // ---------------------------------------------------------------------------
  // GET THE STATION POINT FROM FILE
  // ---------------------------------------------------------------------------
  std::ifstream station_file;
  station_file.open("/home/helio/catkin_ws/src/TOOLING4G/franka_udrilling/co_manipulation_data/station");

  double X, Y, Z;
  if(station_file.is_open()){
    station_file >> X >> Y >> Z;
  }
  else{
    std::cout << "Error opening the station file!" << std::endl;
    return(0);
  }
  station_file.close();
  Eigen::Vector3d S;  // robot station
  S << X, Y, Z;
  // std::cout << station.transpose() << std::endl;

  geometry_msgs::Point p_station;
  p_station.x = S(0);
  p_station.y = S(1);
  p_station.z = S(2);
  points.points.push_back(p_station);


  // ---------------------------------------------------------------------------
  // GET THE DESIRED ROTATION FROM FILE
  // ---------------------------------------------------------------------------
  std::ifstream orientation_file;
  orientation_file.open("/home/helio/catkin_ws/src/TOOLING4G/franka_udrilling/co_manipulation_data/mould_orientation");

  double qx, qy, qz, qw;
  if(orientation_file.is_open()){
    orientation_file >> qx >> qy >> qz >> qw;
  }
  else{
    std::cout << "Error opening the orientation file!" << std::endl;
    return(0);
  }
  orientation_file.close();
  Eigen::Quaterniond Qd;  // desired Quaternion
  Qd.vec()[0] = qx;
  Qd.vec()[1] = qy;
  Qd.vec()[2] = qz;
  Qd.w() = qw;
  // std::cout << Qd.coeffs() << std::endl;

  Eigen::Matrix3d Rd(Qd); // desired Rotation
  // std::cout << Rd << std::endl;


  // ---------------------------------------------------------------------------
  // GET MOULD POINTS FROM FILE
  // ---------------------------------------------------------------------------
  Eigen::MatrixXd P;  // matrix to save the mould points
  std::ifstream points_file;
  points_file.open("/home/helio/catkin_ws/src/TOOLING4G/franka_udrilling/co_manipulation_data/mould_points");

  int n_points = 0;
  P.resize(3, n_points + 1);
  if(points_file.is_open()){
    while(points_file >> X >> Y >> Z){
      // save the values in the matrix
      P.conservativeResize(3, n_points + 1);
      P(0, n_points) = X;
      P(1, n_points) = Y;
      P(2, n_points) = Z;
      n_points++;
    }
  }
  else{
    std::cout << "\nError opening the mould points file!" << std::endl;
    return(0);
  }
  points_file.close();
  // std::cout << P.transpose() << std::endl;

  geometry_msgs::Point p_points;
  int i = 0;
  while(i < n_points){
    p_points.x = P(0, i);
    p_points.y = P(1, i);
    p_points.z = P(2, i);
    points.points.push_back(p_points);
    i++;
  }


  // ---------------------------------------------------------------------------
  // GET INITIAL POSE
  // ---------------------------------------------------------------------------
  Eigen::Matrix4d O_T_EE_i;
  Eigen::VectorXd pose_i(7,1);
  O_T_EE_i = panda.O_T_EE;
  pose_i = panda.robot_pose(O_T_EE_i);


  // ---------------------------------------------------------------------------
  // TRAJECTORY TO FIRST POINT
  // ---------------------------------------------------------------------------
  // position
  Eigen::Vector3d pi, pf;
  pi << pose_i[0], pose_i[1], pose_i[2];
  pf << S(0), S(1), pi(2);
  double ti = 2.0;
  double tf = 5.0;
  double t = 0.0;
  double delta_t = 0.001;

  // orientation
  Eigen::Quaterniond oi, of;
  oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
  of.coeffs() << Qd.vec()[0], Qd.vec()[1], Qd.vec()[2], Qd.w();
  double t1 = 0.0;
  double delta_t1 = delta_t/(tf-ti);


  // ---------------------------------------------------------------------------
  // TRAJECTORY UP CONDITIONS
  // ---------------------------------------------------------------------------
  Eigen::Vector3d delta_up;
  delta_up << 0.0, 0.0, 0.3;


  // ---------------------------------------------------------------------------
  // DRILLING TRAJECTORY CONDITIONS
  // ---------------------------------------------------------------------------
  Eigen::Vector3d delta_drill, delta_roof, delta_predrill, delta_goal, delta_limit;
  delta_drill << 0.0, 0.0, 0.001;
  delta_roof << 0.0, 0.0, 0.001;
  delta_predrill << 0.0, 0.0, 0.005;
  delta_goal << 0.0, 0.0, 0.008;  // 0.006
  delta_limit << 0.0, 0.0, 0.015; // 0.012
  Eigen::Vector3d p_roof, p_goal, p_limit;
  p_roof.setZero();
  p_goal.setZero();
  p_limit.setZero();
  double max_force_limit = 12.0;
  double min_force_limit = 0.0;


  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  Eigen::Vector3d position_d(pi);
  Eigen::Quaterniond orientation_d(oi);

  int flag_drilling = 0;
  int flag_print = 0;
  int flag_interrupt = 0;
  int n_points_done = 0;
  double result = 0.0;

  ros::Rate loop_rate(1000);
  int count = 0;
  while(ros::ok()){

    switch (flag_drilling) { ///////////////////////////////////////////////////

      // -----------------------------------------------------------------------
      case MOVE2STATION:
        if(flag_print == 0){
          std::cout << CLEANWINDOW << "ROBOT IS MOVING TO THE STATION..." << std::endl;
          flag_print = 1;
        }

        // --> MOVE TO STATION <--
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
          if ( t1 <= 1.0 ){
            orientation_d = oi.slerp(t1, of);
            orientation_d.normalize();
          }
          t1 = t1 + delta_t1;
        }
        else if(t > tf){
          flag_drilling = STATIONDOWN;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << S(0), S(1), S(2);
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case STATIONDOWN:
        // --> STATION DOWN <--
        ti = 0.0;
        tf = 5.0;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = STATIONUP;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << pi - Rd*delta_up;
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case STATIONUP:
        // --> STATION UP <--
        ti = 0.0;
        tf = 4.0;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = MOVE2POINT;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << P(0, n_points_done), P(1, n_points_done), pi(2);
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case MOVE2POINT:
        if(flag_print == 1){
          std::cout << CLEANWINDOW << "ROBOT IS MOVING TO A MOULD POINT..." << std::endl;
          flag_print = 2;
        }

        // --> MOVE TO MOULD POINT <--
        ti = 0.0;
        tf = 4.0;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = POINTDOWN;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << P(0, n_points_done), P(1, n_points_done), P(2, n_points_done);
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case POINTDOWN:
        // --> POINT DOWN <--
        ti = 0.0;
        tf = 4.0;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = PREDRILL;
          pi << P(0, n_points_done), P(1, n_points_done), P(2, n_points_done);
          pf << pi + Rd*delta_predrill;
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case PREDRILL:
        // --> PRE DRILL <--
        ti = 0.0;
        tf = 15.0;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          if(flag_print == 2){
            std::cout << CLEANWINDOW << "ROBOT IS READY, PLEASE PRESS BUTTON <1> OF SPACENAV TO START DRILLING!" << std::endl;
            flag_print = 3;
          }
          // if( panda.spacenav_button_1 == 1 ){
          if( panda.K_F_ext_hat_K[2] > 5.0 ){
            flag_drilling = DRILL;
            pi << position_d;
            pf << pi + Rd*delta_drill;
            p_roof << pi + Rd*delta_roof;
            p_goal << pi + Rd*delta_goal;
            p_limit << pi + Rd*delta_limit;
            t = 0;  // reset time
          }
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case DRILL:
        if(flag_print == 3){
          std::cout << CLEANWINDOW << "ROBOT IS DRILLING, IF YOU WOULD LIKE TO STOP PRESS SPACENAV BUTTON <2>..."  << std::endl;
          flag_print = 4;
        }

        O_T_EE_i = panda.O_T_EE;
        pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
        result = pose_i(2) - p_goal(2);
        if( result > 0.0 || (panda.K_F_ext_hat_K[2] > min_force_limit) ){
          // --> DRILL <--
          ti = 0.0;
          tf = 0.6;
          if( (t >= ti) && (t <= tf) ){
            position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
          }
          else if(t > tf){
            flag_drilling = DRILLUP;
            pi << position_d;
            pf << p_roof;
            t = 0;  // reset time
          }
          t = t + delta_t;
        }
        else{
          flag_drilling = CHANGEPOINT;
          flag_print = 4;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          delta_up << 0.0, 0.0, 0.1;
          pf << pi - Rd*delta_up;
          t = 0;
        }

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case DRILLUP:
        // --> DRILL UP <--
        ti = 0.0;
        tf = 0.5;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = DRILLDOWN;
          pf << pi;
          pi << p_roof;
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case DRILLDOWN:
        // --> DRILL DOWN <--
        ti = 0.0;
        tf = 0.5;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = DRILL;
          pi << position_d;
          if( (pi(2) < p_limit(2)) || (panda.K_F_ext_hat_K[2] > max_force_limit) ){
            pf << pi;
          }
          else{
            pf << pi + Rd*delta_drill;
          }
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case CHANGEPOINT:
        if(flag_print == 4){
          std::cout << CLEANWINDOW << "THE HOLE IS COMPLETE AND THE ROBOT IS MOVING UP!" << " result(m): " << result << std::endl;
          flag_print = 0;
        }

        // --> MOVING UP <--
        ti = 0.0;
        tf = 4.0;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){

          if(n_points_done < n_points-1){
            n_points_done++;  // next point
          }
          else{
            if(flag_print == 0){
              std::cout << CLEANWINDOW << "ALL HOLES COMPLETE!" << std::endl;
              flag_print = 5;
            }
            return 0;
          }

          flag_drilling = MOVE2STATION;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << S(0), S(1), pi[2];
          delta_up << 0.0, 0.0, 0.3;
          ti = 0.0;
          tf = 4.0;
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case INTERRUPT:

        if(flag_interrupt == 0){
          std::cout << CLEANWINDOW << "PROGRAM INTERRUPTED! IF YOU WOULD LIKE TO CONTINUE, PLEASE PRESS SPACENAV BUTTON <1>..." << std::endl;
          flag_interrupt = 1;
        }

        if(panda.spacenav_button_1 == 1){
          flag_drilling = CHANGEPOINT;
          flag_interrupt = 0;
          flag_print = 4;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          delta_up << 0.0, 0.0, 0.1;
          pf << pi - Rd*delta_up;
          t = 0;
        }

        break;

    } //////////////////////////////////////////////////////////////////////////

    // std::cout << CLEANWINDOW << position_d << std::endl;
    // std::cout << CLEANWINDOW << orientation_d.coeffs() << std::endl;
    panda.posePublisherCallback(marker_pose, position_d, orientation_d);

    // -------------------------------------------------------------------------
    // TF AND VISUALIZATION MARKERS
    // -------------------------------------------------------------------------
    // Draw the station tf
    station_tf.setOrigin( tf::Vector3(S(0), S(1), S(2)) );
    station_tf.setRotation( tf::Quaternion(Qd.vec()[0], Qd.vec()[1], Qd.vec()[2], Qd.w()) );
    station_br.sendTransform(tf::StampedTransform(station_tf, ros::Time::now(), "/panda_link0", "/station"));

    // Draw the panda EE desired transform
    pandaEEd_tf.setOrigin( tf::Vector3(position_d(0), position_d(1), position_d(2)) );
    pandaEEd_tf.setRotation( tf::Quaternion(orientation_d.vec()[0], orientation_d.vec()[1], orientation_d.vec()[2], orientation_d.w()) );
    pandaEEd_br.sendTransform(tf::StampedTransform(pandaEEd_tf, ros::Time::now(), "/panda_link0", "/panda_EE_d"));

    // Draw the mould transform
    mould_tf.setOrigin( tf::Vector3(P(0, n_points_done), P(1, n_points_done), P(2, n_points_done)) );
    mould_tf.setRotation( tf::Quaternion(Qd.vec()[0], Qd.vec()[1], Qd.vec()[2], Qd.w()) );
    mould_br.sendTransform(tf::StampedTransform(mould_tf, ros::Time::now(), "/panda_link0", "/mould"));

    // Draw the points
    marker_pub.publish(points);
    // -------------------------------------------------------------------------

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
}
