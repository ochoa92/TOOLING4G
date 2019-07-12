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
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  

  ros::Rate loop_rate(1000);
  while (ros::ok()){

    
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}