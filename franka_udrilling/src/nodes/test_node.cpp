// =============================================================================
// Name        : test_node.cpp
// Author      : Hélio Ochoa
// Description :
// =============================================================================
#include <franka_udrilling/udrilling_state.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>



int main(int argc, char **argv){

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    franka_udrilling::uDrillingState panda(nh);

    


    // =============================================================================
    //                                  MAIN LOOP
    // =============================================================================
        
    ros::Rate loop_rate(1000);
    while(ros::ok()){

        
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
