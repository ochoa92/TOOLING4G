// =============================================================================
// Name        : user_pattern_node.cpp
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
#define INTERRUPT 3
// -----------------------------------------------------------------------------

int main(int argc, char** argv) {

  ros::init(argc, argv, "user_pattern_node");

  ros::NodeHandle nh;
  franka_polishing::Spacenav panda(nh);

  // ---------------------------------------------------------------------------
  // PATTERN
  // ---------------------------------------------------------------------------
  Eigen::MatrixXd position;  // matrix to save the robot positions in Base-frame
  Eigen::MatrixXd orientation;  // matrix to save the robot orientations in Base-frame
  std::ifstream infile;
  std::string path;
  path = "/home/panda/kst/polishing/patterns/polishing_pattern";
  infile.open(path);

  double time, px, py, pz, qx, qy, qz, qw, Fx_EE, Fy_EE, Fz_EE, Fx_O, Fy_O, Fz_O;
  std::string line;
  getline(infile, line);  // first line
  getline(infile, line);  // second line
  int column = 0;
  position.resize(3, column + 1);
  orientation.resize(4, column + 1);
  if(infile.is_open()){
    while(infile >> time >> px >> py >> pz >> qx >> qy >> qz >> qw >> Fx_EE >> Fy_EE >> Fz_EE >> Fx_O >> Fy_O >> Fz_O){
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
  infile.close();
  //std::cout << position.transpose() << std::endl;
  //std::cout << orientation.transpose() << std::endl;

  Eigen::Quaterniond Qpattern;
  Qpattern.coeffs() = orientation.col(0);
  Eigen::Matrix3d Rpattern(Qpattern);

  // ---------------------------------------------------------------------------
  // GET THE OFFSETS BETWEEN POSES
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
  // MOULD POINTS
  // ---------------------------------------------------------------------------
  std::ifstream file_plane;
  std::string path_plane;
  double x1, x2, x3, x4;
  double y1, y2, y3, y4;
  double z1, z2, z3, z4;
  path_plane = "/home/panda/kst/polishing/planes/mould_points";
  file_plane.open(path_plane);
  if(file_plane.is_open()){
    file_plane >> x1 >> y1 >> z1;
    file_plane >> x2 >> y2 >> z2;
    file_plane >> x3 >> y3 >> z3;
    file_plane >> x4 >> y4 >> z4;
  }
  else{
    std::cout << "Error open the file!" << std::endl;
    return(0);
  }
  file_plane.close();

  Eigen::Matrix<double, 4, 3> P;
  P << x1, y1, z1,
       x2, y2, z2,
       x3, y3, z3,
       x4, y4, z4;
  //std::cout << P << std::endl;

  Eigen::Vector3d P1(P.row(0));
  Eigen::Vector3d P2(P.row(1));
  Eigen::Vector3d P3(P.row(2));
  Eigen::Vector3d P4(P.row(3));

  // compute the vectors of the desired rotation matrix
  Eigen::Vector3d nx( (P4-P1)/((P4-P1).norm()) );
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
  //std::cout << Rmould << std::endl;

  // mould orientation in Quaternion unit
  Eigen::Quaterniond Qmould(Rmould);

  // compute the desired rotation in all points of plane
  Eigen::Matrix3d Rpoints(Rmould * Rpattern);
  Eigen::Quaterniond Qpoints(Rpoints);


  // ---------------------------------------------------------------------------
  // MOULD WORK SPACE
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

  // build a vector with all points of the mould work space
  int Nx = 7;
  int Ny = 5;
  double step_x = 1/(double)Nx;
  double step_y = 1/(double)Ny;

  Eigen::Vector3d delta_synthetic;  // create a synthetic reference
  delta_synthetic << 0.0, 0.0, -0.003;
  Eigen::Vector3d P1_synthetic(P1 + delta_synthetic);
  Eigen::Vector3d P2_synthetic(P2 + delta_synthetic);
  Eigen::Vector3d P4_synthetic(P4 + delta_synthetic);
  Eigen::Vector3d vec_x = P4_synthetic - P1_synthetic;
  Eigen::Vector3d vec_y = P2_synthetic - P1_synthetic;

  Eigen::MatrixXd Xd;
  Xd.resize(3, (Nx+1)*(Ny+1));
  int n = 0;  // number of columns
  for(int i = 0; i <= Nx; ++i){
    for(int j = 0; j <= Ny; ++j){
      Xd(0, n) = P1_synthetic(0) + (vec_x(0)*step_x*i + vec_y(0)*step_y*j);
      Xd(1, n) = P1_synthetic(1) + (vec_x(1)*step_x*i + vec_y(1)*step_y*j);
      Xd(2, n) = P1_synthetic(2) + (vec_x(2)*step_x*i + vec_y(2)*step_y*j);
      ++n;
    }
  }
  // std::cout << Xd.transpose() << std::endl;

  geometry_msgs::Point p, l;  // points and lines
  int i = 0;
  while(i < (Nx+1)*(Ny+1)){
    p.x = Xd(0, i);
    p.y = Xd(1, i);
    p.z = Xd(2, i);
    points.points.push_back(p);
    i++;
  }


  // ---------------------------------------------------------------------------
  // tf broadcaster
  // ---------------------------------------------------------------------------
  tf::TransformBroadcaster br, br_d;
  tf::Transform transform, transform_d;


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
  int num_cols = n-1;
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

  int flag_pattern = 0;
  int flag_print = 0;
  int flag_interrupt = 0;

  ros::Rate loop_rate(1000);
  while (ros::ok()){

    switch (flag_pattern) { ////////////////////////////////////////////////////

      // -----------------------------------------------------------------------
      case MOVE2POINT:
      {
        if(flag_print == 0){
          std::cout << CLEANWINDOW << "ROBOT IS MOVING TO A MOULD POINT..." << std::endl;
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
        }
        t = t + delta_t;

        new_position_d = position_d;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_pattern = INTERRUPT;
        }

        break;
      }


      // -----------------------------------------------------------------------
      case PATTERN:
      {
        if(flag_print == 1){
          std::cout << CLEANWINDOW << "PATTERN IS EXECUTING, IF YOU WOULD LIKE TO STOP PRESS SPACENAV BUTTON <2>..." << std::endl;
          flag_print = 2;
        }

        // -------------------------------------------------------------------------
        // PATTERN LOOP
        // -------------------------------------------------------------------------
        int count = 0;
        ros::Rate loop_rate(1000);
        while (count < column-1){

          Eigen::Vector3d mould_offset;
          mould_offset << OFFSET(0, count),
                          OFFSET(1, count),
                          OFFSET(2, count);

          position_d = position_d + mould_offset;

          // distances between the pattern points and the 4 points that delimit the plane
          Eigen::Vector3d l1P(position_d - P1);
          Eigen::Vector3d l2P(position_d - P2);
          Eigen::Vector3d l3P(position_d - P3);
          Eigen::Vector3d l4P(position_d - P4);

          // signal between the 2 vectors
          double signal1 = l1P.dot(l12);
          double signal2 = l2P.dot(l23);
          double signal3 = l3P.dot(l34);
          double signal4 = l4P.dot(l41);
          if( (signal1 >= 0.0) && (signal2 >= 0.0) && (signal3 >= 0.0) && (signal4 >= 0.0) ){
            new_position_d = position_d;
          }

          orientation_d.coeffs() = Qpoints.coeffs();

          //std::cout << CLEANWINDOW << new_position_d << std::endl;
          // std::cout << CLEANWINDOW << orientation_d.coeffs() << std::endl;
          panda.posePublisherCallback(marker_pose, new_position_d, orientation_d);

          // draw the lines of the polished area
          if(count % 100 == 0){
            l.x = new_position_d[0];
            l.y = new_position_d[1];
            l.z = new_position_d[2];
            line_strip.points.push_back(l);
            marker_pub.publish(line_strip);
          }

          // -------------------------------------------------------------------
          // TF AND VISUALIZATION MARKERS
          // -------------------------------------------------------------------
          // Draw the points of the mould work space
          marker_pub.publish(points);

          // Draw the mould transform
          transform.setOrigin( tf::Vector3(P1(0), P1(1), P1(2)) );
          transform.setRotation( tf::Quaternion(Qmould.vec()[0], Qmould.vec()[1], Qmould.vec()[2], Qmould.w()) );
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/mould"));
          marker_pub.publish(points); // Draw the points of the mould work space

          // Draw the desired EE transform
          transform_d.setOrigin( tf::Vector3(new_position_d(0), new_position_d(1), new_position_d(2)) );
          transform_d.setRotation( tf::Quaternion(orientation_d.vec()[0], orientation_d.vec()[1], orientation_d.vec()[2], orientation_d.w()) );
          br_d.sendTransform(tf::StampedTransform(transform_d, ros::Time::now(), "/panda_link0", "/panda_EE_d"));
          // -------------------------------------------------------------------

          if(panda.spacenav_button_2 == 1)
            break;

          ros::spinOnce();
          loop_rate.sleep();
          count++;
        }// --------------------------------------------------------------------

        flag_pattern = MOVEUP;
        O_T_EE_i = panda.O_T_EE;
        pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
        pi << pose_i[0], pose_i[1], pose_i[2];
        pf << pi + delta_up;
        t = 0;  // reset the time

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_pattern = INTERRUPT;
        }

        break;
      }


      // -----------------------------------------------------------------------
      case MOVEUP:
      {
        if(flag_print == 2){
          std::cout << CLEANWINDOW << "PATTERN WAS EXECUTED!" << std::endl;
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
          }
          else{
            std::cout << CLEANWINDOW << "MOULD IS ALREADY POLISHED!" << std::endl;
            break;
          }

          flag_pattern = MOVE2POINT;
          O_T_EE_i = panda.O_T_EE;  // get current pose
          pose_i = panda.robot_pose(O_T_EE_i);
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << Xd(0, num_cols), Xd(1, num_cols), Xd(2, num_cols);
          t = 0.0;
          ti = 0.0;
          tf = 5.0;
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_pattern = INTERRUPT;
        }

        break;
      }


      // -----------------------------------------------------------------------
      case INTERRUPT:
      {
        if(flag_interrupt == 0){
          std::cout << CLEANWINDOW << "PROGRAM INTERRUPTED! IF YOU WOULD LIKE TO CONTINUE, PLEASE PRESS SPACENAV BUTTON <1>..." << std::endl;
          flag_interrupt = 1;
        }

        if(panda.spacenav_button_1 == 1){
          flag_pattern = MOVEUP;
          flag_interrupt = 0;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << pi + delta_up;
          t = 0;
        }
      }

        break;

    } //////////////////////////////////////////////////////////////////////////

    // std::cout << CLEANWINDOW << position_d << std::endl;
    // std::cout << CLEANWINDOW << orientation_d.coeffs() << std::endl;
    panda.posePublisherCallback(marker_pose, position_d, orientation_d);

    // -------------------------------------------------------------------------
    // TF AND VISUALIZATION MARKERS
    // -------------------------------------------------------------------------
    // Draw the points of the mould work space
    marker_pub.publish(points);

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
