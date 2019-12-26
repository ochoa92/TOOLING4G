// =============================================================================
// Name        : simulation_test_node.cpp
// Author      : Hélio Ochoa
// Description : 
// =============================================================================
#include <franka_simulation/spacenav.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "simulation_test_node");
    ros::NodeHandle nh;
    franka_simulation::Spacenav panda(nh);

    Eigen::Vector2d fingers_position;
    fingers_position << 0.5, 0.5;
    

    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    ros::Rate loop_rate(1000);
    int count = 0;
    while (ros::ok()){

        panda.moveFingersCallback(fingers_position);
                
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
        break;

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}