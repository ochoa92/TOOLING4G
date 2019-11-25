// =============================================================================
// Name        : polishing_test_node.cpp
// Author      : Hélio Ochoa
// Description :
// =============================================================================
#include <franka_polishing/spacenav.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "polishing_test_node");

  ros::NodeHandle nh;
  franka_polishing::Spacenav panda(nh);
  geometry_msgs::PoseStamped marker_pose;

 
  // ---------------------------------------------------------------------------
  // DEFINE WORK-SPACE LIMITS
  // ---------------------------------------------------------------------------
  std::ifstream ws_file;
  std::string ws_path;
  double x, y, z;
  Eigen::MatrixXd P;
  ws_path = "/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/mold_workspace";
  ws_file.open(ws_path);
  int n_ws = 0;
  P.resize(n_ws + 1, 3);
  if(ws_file.is_open()){
      while(ws_file >> x >> y >> z){
          // save the values in the matrix
          P.conservativeResize(n_ws + 1, 3);
          P(n_ws, 0) = x;
          P(n_ws, 1) = y;
          P(n_ws, 2) = z;
          n_ws++;
      }
  }
  else{
      std::cout << "Error open the file!" << std::endl;
      return(0);
  }
  ws_file.close();
  // std::cout << P << std::endl;

  Eigen::Vector3d P1(P.row(0));
  Eigen::Vector3d P2(P.row(1));
  Eigen::Vector3d P3(P.row(2));
  Eigen::Vector3d P4(P.row(3));

  Eigen::Matrix3d Rmold;
  Rmold = panda.points2Rotation(P1, P2, P4);
  //std::cout << Rmold << std::endl;

  // mould orientation in Quaternion unit
  Eigen::Quaterniond Qmold(Rmold);

  // ---------------------------------------------------------------------------
  // TF BROADCASTER
  // ---------------------------------------------------------------------------
  tf::TransformBroadcaster mold_br;
  tf::Transform mold_tf;
  
  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  ros::Rate loop_rate(1000);
  while (ros::ok()){

    // Draw the mold transform
    mold_tf.setOrigin( tf::Vector3(P1(0), P1(1), P1(2)) );
    mold_tf.setRotation( tf::Quaternion(Qmold.vec()[0], Qmold.vec()[1], Qmold.vec()[2], Qmold.w()) );
    mold_br.sendTransform(tf::StampedTransform(mold_tf, ros::Time::now(), "/panda_link0", "/mold"));

    
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}