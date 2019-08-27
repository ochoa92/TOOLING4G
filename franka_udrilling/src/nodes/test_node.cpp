// =============================================================================
// Name        : test_node.cpp
// Author      : Hélio Ochoa
// Description :
// =============================================================================
#include <franka_udrilling/spacenav.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    franka_udrilling::Spacenav panda(nh);

    geometry_msgs::PoseStamped marker_pose;

    // system("rosrun dynamic_reconfigure dynparam set /dynamic_reconfigure_compliance_param_node Kpx 0.0");
    // system("rosrun dynamic_reconfigure dynparam set /dynamic_reconfigure_compliance_param_node Kpz 0.0");

    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    ros::Rate loop_rate(1000);
    int count = 0;
    while(ros::ok()){


        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            break;

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
