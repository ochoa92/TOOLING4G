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

    // ---------------------------------------------------------------------------
    // Get initial transformation matrix
    // ---------------------------------------------------------------------------
    Eigen::Matrix4d O_T_EE;
    O_T_EE = panda.O_T_EE;
    Eigen::VectorXd pose(7,1);
    pose = panda.robotPose(O_T_EE);

    // ---------------------------------------------------------------------------
    // polynomial3Trajectory initial conditions
    // ---------------------------------------------------------------------------
    Eigen::Vector3d pi;
    pi << pose[0], pose[1], pose[2];
    Eigen::Vector3d delta_p;
    delta_p << 0.0, 0.0, 0.1;
    Eigen::Vector3d pf;
    pf << pi + delta_p;
    double ti = 3.0;
    double tf = 5.0;
    double t = 0.0;
    double delta_t = 0.001;

    // ---------------------------------------------------------------------------
    // ellipseTrajectory initial conditions
    // ---------------------------------------------------------------------------
    double t_ellipse = 30.0;
    double a = 0.1;
    double b = 0.1;
    double T = 10.0;
    std::string axis = "XY";

    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    Eigen::Vector3d pd;
    pd << pose[0], pose[1], pose[2];
    Eigen::Quaterniond od;
    od.coeffs() << pose[3], pose[4], pose[5], pose[6];

    int flag_trajectory = 0;
    ros::Rate loop_rate(1000);
    int count = 0;
    while (ros::ok()){

        // polynomial3Trajectory
        if(flag_trajectory == 0){
            if( (t >= ti) && (t <= tf) ){
                pd = panda.polynomial3Trajectory(pi, pf, ti, tf, t);
            }
            else if(t > tf){
                t = 0;
                flag_trajectory = 1;
                pi = pd;
            }
            t = t + delta_t;
        }

        // ellipseTrajectory
        if(flag_trajectory == 1){
            if( t <= t_ellipse ){
                pd = panda.ellipseTrajectory(pi, a, b, T, t, axis);
            }
            else if(t > t_ellipse){
                t = 0;
                flag_trajectory = 2;
                pi = pd;
                pf = pi - delta_p;
            }
            t = t + delta_t;
        }

        // lineTrajectory
        if(flag_trajectory == 2){
            if( t <= 1 ){
                pd = panda.lineTrajectory(pi, pf, t);
            }
            else if(t > 1){

            }
            t = t + 0.0005;
        }


        // std::cout << CLEANWINDOW << "\npd:\n" << pd << std::endl;
        // std::cout << CLEANWINDOW << "\nod:\n" << od.vec() << "\n" << od.w() << std::endl;
        panda.posePublisherCallback(pd, od);

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            break;

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
