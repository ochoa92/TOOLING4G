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

    

    Eigen::Matrix4d O_T_EE;
    Eigen::VectorXd pose(7,1);

    int result = 0;
    double Fm = 0.0;

    double d = 0.0;
    double dt = 0.2;
    double c = 0.5;

    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    ros::Rate loop_rate(1000);
    while (ros::ok()){


        O_T_EE = panda.O_T_EE;
        pose = panda.robot_pose(O_T_EE);
        d = pose(2);

        if(d < dt){
            result = 1;
        }
        else{
            result = 0;
        }

        if(result == 1){
            Fm = ( 1/(1+std::exp(-c*(-(d-dt)))) ) * panda.K_F_ext_hat_K[2];
        }
        else{
            Fm = 0.0;
        }
        
        std::cout << CLEANWINDOW << "Result: " << result << " | Fm = " << Fm << std::endl;
        
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            break;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}