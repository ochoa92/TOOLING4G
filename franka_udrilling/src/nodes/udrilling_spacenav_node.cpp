// =============================================================================
// Name        : udrilling_test_node.cpp
// Author      : Hélio Ochoa
// Description : Allows user to control the Panda robot pose with space-mouse.
// =============================================================================
#include <franka_udrilling/spacenav.h>
#include <tf/transform_broadcaster.h>

using namespace franka_udrilling;

geometry_msgs::PoseStamped marker_pose;

int main(int argc, char **argv){

  ros::init(argc, argv, "udrilling_spacenav_node");
  ros::NodeHandle nh;
  franka_udrilling::Spacenav panda(nh);

  // ---------------------------------------------------------------------------
  // tf broadcaster
  // ---------------------------------------------------------------------------
  tf::TransformBroadcaster br_d;
  tf::Transform transform_d;

  // ---------------------------------------------------------------------------
  // Get initial transformation matrix
  // ---------------------------------------------------------------------------
  Eigen::Matrix4d O_T_EE_d;
  O_T_EE_d = panda.O_T_EE;

  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  // open file to save pattern data
  std::ofstream file;
  std::string path;
  path = "/home/helio/kst/udrilling/pattern/spacenav_pattern";
  file.open(path);
  file << " p_xd p_yd p_zd Qxd Qyd Qzd Qwd\n";

  ros::Rate loop_rate(1000);
  int count = 0;
  while (ros::ok()){

    panda.MotionControl(O_T_EE_d);
    // std::cout << CLEANWINDOW << "\nposition_d:\n" << panda.position_d << std::endl;
    //std::cout << CLEANWINDOW << "\norientation_d:\n" << panda.orientation_d.vec() << "\n" << panda.orientation_d.w() << std::endl;

    panda.posePublisherCallback(marker_pose, panda.position_d, panda.orientation_d);

    // ---------------------------------------------------------------------
    // Draw the desired transform
    // ---------------------------------------------------------------------
    transform_d.setOrigin( tf::Vector3(panda.position_d(0), panda.position_d(1), panda.position_d(2)) );
    transform_d.setRotation( tf::Quaternion(panda.orientation_d.vec()[0], panda.orientation_d.vec()[1], panda.orientation_d.vec()[2], panda.orientation_d.w()) );
    br_d.sendTransform(tf::StampedTransform(transform_d, ros::Time::now(), "/panda_link0", "/panda_EE_d"));

    // save pattern
    file << " " << panda.position_d[0] << " "
                << panda.position_d[1] << " "
                << panda.position_d[2] << " "
                << panda.orientation_d.vec()[0] << " "
                << panda.orientation_d.vec()[1] << " "
                << panda.orientation_d.vec()[2] << " "
                << panda.orientation_d.w() << "\n";

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  file.close();

  return 0;
}
