// =============================================================================
// Name        : nflat_surfaces_node.cpp
// Author      : Hélio Ochoa
// Description :
// =============================================================================
#include <franka_polishing/spacenav.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::PoseStamped marker_pose;

// STATE MACHINE --------------------------------------------------------------
#define MOVE2POINT 0
#define PATTERN 1
#define MOVEUP 2
#define CHOOSEPLANE 3
#define INTERRUPT 4
// -----------------------------------------------------------------------------

int main(int argc, char** argv) {

  ros::init(argc, argv, "nflat_surfaces_node");

  ros::NodeHandle nh;
  franka_polishing::Spacenav panda(nh);


  // ---------------------------------------------------------------------------
  // VIZUALIZATIONS MARKERS INITIALIZATION
  // ---------------------------------------------------------------------------
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


  // ---------------------------------------------------------------------------
  // TF BROADCASTER
  // ---------------------------------------------------------------------------
  tf::TransformBroadcaster br, br_d;
  tf::Transform transform, transform_d;


  // ---------------------------------------------------------------------------
  // GET THE CO-MANIPULATION PATTERN FROM A FILE
  // ---------------------------------------------------------------------------
  Eigen::MatrixXd position;  // matrix to save the robot positions in Base-frame
  Eigen::MatrixXd orientation;  // matrix to save the robot orientations in Base-frame
  std::ifstream pattern_file;
  std::string pattern_path;
  pattern_path = "/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/pattern";
  pattern_file.open(pattern_path);

  double time, px, py, pz, qx, qy, qz, qw;
  std::string line;
  getline(pattern_file, line);  // first line
  getline(pattern_file, line);  // second line
  int column = 0;
  position.resize(3, column + 1);
  orientation.resize(4, column + 1);
  if(pattern_file.is_open()){
    while(pattern_file >> time >> px >> py >> pz >> qx >> qy >> qz >> qw){
      // save the values in the matrix
      position.conservativeResize(3, column + 1);
      orientation.conservativeResize(4, column + 1);
      position(0, column) = px;
      position(1, column) = py;
      position(2, column) = pz;
      orientation(0, column) = qx;
      orientation(1, column) = qy;
      orientation(2, column) = qz;
      orientation(3, column) = qw;
      column++;
    }
  }
  else{
    std::cout << "\nError open the file!" << std::endl;
    return(0);
  }
  pattern_file.close();
  //std::cout << position.transpose() << std::endl;
  //std::cout << orientation.transpose() << std::endl;

  Eigen::Quaterniond Qpattern;
  Qpattern.coeffs() = orientation.col(0);
  Eigen::Matrix3d Rpattern(Qpattern);


  // ---------------------------------------------------------------------------
  // COMPUTE OFFSETS BETWEEN POSITIONS
  // ---------------------------------------------------------------------------
  Eigen::MatrixXd OFFSET;
  OFFSET.resize(3, column-1);
  for(int i = 0; i < column-1; i++){
    OFFSET(0, i) = position(0, i+1) - position(0, i);
    OFFSET(1, i) = position(1, i+1) - position(1, i);
    OFFSET(2, i) = position(2, i+1) - position(2, i);
  }
  //std::cout << OFFSET.transpose() << std::endl;


  // ---------------------------------------------------------------------------
  // GET MOULD POINTS FROM A FILE
  // ---------------------------------------------------------------------------
  std::ifstream file_plane;
  std::string path_plane;
  double x, y, z;
  Eigen::MatrixXd P;
  path_plane = "/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/plane_points";
  file_plane.open(path_plane);
  int n_points = 0;
  P.resize(3, n_points + 1);
  if(file_plane.is_open()){
    while(file_plane >> x >> y >> z){
      // save the values in the matrix
      P.conservativeResize(3, n_points + 1);
      P(0, n_points) = x;
      P(1, n_points) = y;
      P(2, n_points) = z;
      n_points++;
    }
  }
  else{
    std::cout << "Error open the file!" << std::endl;
    return(0);
  }
  file_plane.close();
  // std::cout << P << std::endl;
  int n_planes = n_points/4;

  int n = 0;
  Eigen::Vector3d P1(P.col(n));
  Eigen::Vector3d P2(P.col(n+1));
  Eigen::Vector3d P3(P.col(n+2));
  Eigen::Vector3d P4(P.col(n+3));

  Eigen::Matrix3d Rmould;
  Rmould = panda.points2Rotation(P1, P2, P4);
  //std::cout << Rmould << std::endl;

  // mould orientation in Quaternion unit
  Eigen::Quaterniond Qmould(Rmould);

  // compute the desired rotation in all points of plane
  Eigen::Matrix3d Rpoints(Rmould * Rpattern);
  Eigen::Quaterniond Qpoints(Rpoints);

  // build a vector with all points of the mould work space
  int Nx = 5;
  int Ny = 5;
  Eigen::Vector3d delta_synthetic;
  delta_synthetic << 0.0, 0.0, 0.0;
  Eigen::MatrixXd Xd;
  Xd = panda.moldWorkSpace(P1, P2, P4, delta_synthetic, Nx, Ny);
  // std::cout << Xd.transpose() << std::endl;

  geometry_msgs::Point p, l;  // points and lines
  int n_columns = 0;
  while(n_columns < (Nx+1)*(Ny+1)){
    p.x = Xd(0, n_columns);
    p.y = Xd(1, n_columns);
    p.z = Xd(2, n_columns);
    points.points.push_back(p);
    n_columns++;
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
  Eigen::Vector3d pi, pf;
  int num_cols = n_columns-1;
  pi << pose_i[0], pose_i[1], pose_i[2];
  pf << Xd(0, num_cols), Xd(1, num_cols), Xd(2, num_cols);
  double ti = 1.0;
  double tf = 6.0;
  double t = 0.0;
  double delta_t = 0.001;

  // orientation
  Eigen::Quaterniond oi, of;
  oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
  of.coeffs() << Qpoints.vec()[0], Qpoints.vec()[1], Qpoints.vec()[2], Qpoints.w();
  double t1 = 0.0;
  double delta_t1 = delta_t/(tf-ti);


  // ---------------------------------------------------------------------------
  // TRAJECTORY MOVE UP CONDITIONS
  // ---------------------------------------------------------------------------
  Eigen::Vector3d delta_up;
  delta_up << 0.0, 0.0, 0.1;


  // ---------------------------------------------------------------------------
  // DEFINE WORK-SPACE LIMITS
  // ---------------------------------------------------------------------------
  // distances between the 4 points that delimit the plane
  Eigen::Vector3d l12(P2 - P1);
  Eigen::Vector3d l23(P3 - P2);
  Eigen::Vector3d l34(P4 - P3);
  Eigen::Vector3d l41(P1 - P4);


  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  Eigen::Vector3d position_d(pi);
  Eigen::Quaterniond orientation_d(oi);
  Eigen::Vector3d new_position_d(position_d);

  Eigen::Vector3d mould_offset;
  mould_offset.setZero();

  Eigen::Vector3d l1P, l2P, l3P, l4P;
  l1P.setZero();
  l2P.setZero();
  l3P.setZero();
  l4P.setZero();

  double signal1 = 0.0;
  double signal2 = 0.0;
  double signal3 = 0.0;
  double signal4 = 0.0;

  int flag_pattern = 0;
  int flag_print = 0;
  int flag_interrupt = 0;
  int count = 0;
  int k = 0;
  ros::Rate loop_rate(1000);
  while (ros::ok()){

    switch (flag_pattern) { ////////////////////////////////////////////////////

      // -----------------------------------------------------------------------
      case MOVE2POINT:
        if(flag_print == 0){
          std::cout << CLEANWINDOW << "ROBOT IS MOVING TO A MOLD POINT..." << std::endl;
          flag_print = 1;
        }

        // --> MOVE TO MOULD POINT <--
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
          if ( t1 <= 1.0 ){
            orientation_d = oi.slerp(t1, of);
            orientation_d.normalize();
          }
          t1 = t1 + delta_t1;
        }
        else if(t > tf){
          flag_pattern = PATTERN;
          count = 0;
        }
        t = t + delta_t;

        new_position_d = position_d;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_pattern = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case PATTERN:
        if(flag_print == 1){
          std::cout << CLEANWINDOW << "PATTERN IS EXECUTING, IF YOU WOULD LIKE TO STOP PRESS SPACENAV BUTTON <2>..." << std::endl;
          flag_print = 2;
        }

        // PATTERN
        if (count < column-1){

          mould_offset << OFFSET(0, count), OFFSET(1, count), OFFSET(2, count);
          new_position_d = new_position_d + Rmould * mould_offset;

          // distances between the pattern points and the 4 points that delimit the plane
          l1P = (new_position_d - P1);
          l2P = (new_position_d - P2);
          l3P = (new_position_d - P3);
          l4P = (new_position_d - P4);

          // signal between the 2 vectors
          signal1 = l1P.dot(l12);
          signal2 = l2P.dot(l23);
          signal3 = l3P.dot(l34);
          signal4 = l4P.dot(l41);
          if( (signal1 >= 0.0) && (signal2 >= 0.0) && (signal3 >= 0.0) && (signal4 >= 0.0) ){
            position_d = new_position_d;
          }

        }// --------------------------------------------------------------------
        else{
          flag_pattern = MOVEUP;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << pi + delta_up;
          t = 0;  // reset the time
        }
        count++;

        // draw the lines of the polished area
        if(count % 100 == 0){
          l.x = position_d[0];
          l.y = position_d[1];
          l.z = position_d[2];
          line_strip.points.push_back(l);
          marker_pub.publish(line_strip);
        }

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_pattern = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case MOVEUP:
        if(flag_print == 2){
          std::cout << CLEANWINDOW << "PATTERN WAS EXECUTED AND THE ROBOT IS MOVING UP!" << std::endl;
          flag_print = 0;
        }

        // --> MOVE UP <--
        ti = 0.0;
        tf = 3.0;
        if((t >= ti) && (t <= tf)){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){

          if(num_cols > 0){
            num_cols--;
            flag_pattern = MOVE2POINT;
            O_T_EE_i = panda.O_T_EE;  // get current pose
            pose_i = panda.robot_pose(O_T_EE_i);
            pi << pose_i[0], pose_i[1], pose_i[2];
            pf << Xd(0, num_cols), Xd(1, num_cols), Xd(2, num_cols);
            t = 0.0;
            ti = 0.0;
            tf = 3.0;
          }
          else{
            if(flag_print == 0){
              std::cout << CLEANWINDOW << "MOLD AREA POLISHED!" << std::endl;
              flag_print = 3;
            }

            if(k < n_planes){
              k = k + 1;  // next plane
              n = n + 4;  // next 4 points
              flag_pattern = CHOOSEPLANE;
            }

          }

        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_pattern = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case CHOOSEPLANE:
        if(flag_print == 3){
          std::cout << CLEANWINDOW << "WAIT UNTIL NEW PLANE IS BUILT..." << std::endl;
          flag_print = 0;
        }

        if(k >= n_planes) {
          std::cout << CLEANWINDOW << "ALL MOLD AREAS POLISHED!" << std::endl;
          return 0;
        }
        else{
          P1 = P.col(n);
          P2 = P.col(n+1);
          P3 = P.col(n+2);
          P4 = P.col(n+3);

          Rmould = panda.points2Rotation(P1, P2, P4);
          Qmould = Rmould;

          // compute the desired rotation in all points of plane
          Rpoints = Rmould * Rpattern;
          Qpoints = Rpoints;

          // build a matrix with all points of the mould work space
          Xd = panda.moldWorkSpace(P1, P2, P4, delta_synthetic, Nx, Ny);

          n_columns = 0;
          while(n_columns < (Nx+1)*(Ny+1)){
            p.x = Xd(0, n_columns);
            p.y = Xd(1, n_columns);
            p.z = Xd(2, n_columns);
            points.points.push_back(p);
            n_columns++;
          }

          l12 = (P2 - P1);
          l23 = (P3 - P2);
          l34 = (P4 - P3);
          l41 = (P1 - P4);

          flag_pattern = MOVE2POINT;
          O_T_EE_i = panda.O_T_EE;  // get current pose
          pose_i = panda.robot_pose(O_T_EE_i);
          num_cols = n_columns-1;
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << Xd(0, num_cols), Xd(1, num_cols), Xd(2, num_cols);
          t = 0.0;
          ti = 0.0;
          tf = 3.0;
          oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
          of.coeffs() << Qpoints.vec()[0], Qpoints.vec()[1], Qpoints.vec()[2], Qpoints.w();
          t1 = 0.0;
          delta_t1 = delta_t/(tf-ti);
        }

        break;

      // -----------------------------------------------------------------------
      case INTERRUPT:
        if(flag_interrupt == 0){
          std::cout << CLEANWINDOW << "PROGRAM INTERRUPTED! IF YOU WOULD LIKE TO CONTINUE, PLEASE PRESS SPACENAV BUTTON <1>..." << std::endl;
          flag_interrupt = 1;
        }

        if(panda.spacenav_button_1 == 1){
          flag_print = 2;
          flag_interrupt = 0;
          flag_pattern = MOVEUP;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << pi + delta_up;
          t = 0;
        }

        break;

    }//////////////////////////////////////////////////////////////////////////

    // std::cout << CLEANWINDOW << position_d << std::endl;
    // std::cout << CLEANWINDOW << orientation_d.coeffs() << std::endl;
    panda.posePublisherCallback(marker_pose, position_d, orientation_d);


    // -------------------------------------------------------------------------
    // TF AND VISUALIZATION MARKERS
    // -------------------------------------------------------------------------
    marker_pub.publish(points); // draw the points of the mould work-space

    // Draw the mould transform
    transform.setOrigin( tf::Vector3(P1(0), P1(1), P1(2)) );
    transform.setRotation( tf::Quaternion(Qmould.vec()[0], Qmould.vec()[1], Qmould.vec()[2], Qmould.w()) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/mould"));

    // Draw the desired EE transform
    transform_d.setOrigin( tf::Vector3(position_d(0), position_d(1), position_d(2)) );
    transform_d.setRotation( tf::Quaternion(orientation_d.vec()[0], orientation_d.vec()[1], orientation_d.vec()[2], orientation_d.w()) );
    br_d.sendTransform(tf::StampedTransform(transform_d, ros::Time::now(), "/panda_link0", "/panda_EE_d"));
    // -------------------------------------------------------------------------

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
