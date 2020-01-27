#include <franka_spacenav/spacenav.h>

using namespace franka_spacenav;

geometry_msgs::PoseStamped marker_pose;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spacenav_node");

  ros::NodeHandle nh;
  franka_spacenav::Spacenav franka(nh);

  // variables
  Matrix4d O_T_EE_d;

  // init
  O_T_EE_d = franka.FK;

  std::ofstream file;
  string path;
  path = "/home/panda/kst/franka/robot_patterns/pattern";
  file.open(path);
  file << " p_xd p_yd p_zd Qxd Qyd Qzd Qwd\n";

  // ---------------------------------------------------------------------------
  ros::Rate loop_rate(1000);
  int count = 0;
  while (ros::ok()){

    franka.MotionControl(O_T_EE_d);
    //cout << CLEANWINDOW << "\nposition_d:\n" << franka.position_d << endl;
    //cout << CLEANWINDOW << "\norientation_d:\n" << franka.orientation_d.vec() << "\n" << franka.orientation_d.w() << endl;

    franka.publisherCallback(marker_pose, franka.position_d, franka.orientation_d);

    // save pattern
    file << " " << franka.position_d[0] << " "
                << franka.position_d[1] << " "
                << franka.position_d[2] << " "
                << franka.orientation_d.vec()[0] << " "
                << franka.orientation_d.vec()[1] << " "
                << franka.orientation_d.vec()[2] << " "
                << franka.orientation_d.w() << "\n";

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  file.close();

  return 0;
}
