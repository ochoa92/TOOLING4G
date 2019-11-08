// =============================================================================
// Name        : simulation_spacenav_node.cpp
// Author      : Hélio Ochoa
// Description : Allows user to control the Panda robot pose with space-mouse.
// =============================================================================
#include <franka_simulation/spacenav.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "simulation_spacenav_node");
    ros::NodeHandle nh;
    franka_simulation::Spacenav panda(nh);

    
    // ---------------------------------------------------------------------------
    // Get initial transformation matrix
    // ---------------------------------------------------------------------------
    Eigen::Matrix4d O_T_EE;
    O_T_EE = panda.O_T_EE;

    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    ros::Rate loop_rate(1000);
    int count = 0;
    while (ros::ok()){

        panda.MotionControl(O_T_EE);
        //std::cout << CLEANWINDOW << "\nposition_d:\n" << panda.position_d << std::endl;
        //std::cout << CLEANWINDOW << "\norientation_d:\n" << panda.orientation_d.vec() << "\n" << panda.orientation_d.w() << std::endl;

        panda.posePublisherCallback(panda.position_d, panda.orientation_d);

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
        break;

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
