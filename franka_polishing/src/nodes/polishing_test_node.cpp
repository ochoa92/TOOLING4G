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
    // GET POLYGON VERTICES FROM A FILE
    // ---------------------------------------------------------------------------
    Eigen::MatrixXd vertices;  // matrix to save the polygon vertices
    std::ifstream polygon_file;
    polygon_file.open("/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/polygon_vertices");

    // Vx Vy
    double Vx, Vy;
    std::string line;
    getline(polygon_file, line);  // first line
    int n_vertices = 0;
    vertices.resize(2, n_vertices + 1);
    if(polygon_file.is_open()){
        while(polygon_file >> Vx >> Vy){
            // save the values in the matrix
            vertices.conservativeResize(2, n_vertices + 1);
            vertices(0, n_vertices) = Vx;
            vertices(1, n_vertices) = Vy;
            n_vertices++;
        }
    }
    else{
        std::cout << "\nError open the file!" << std::endl;
        return(0);
    }
    polygon_file.close();
    // std::cout << polygon_vertices.transpose() << std::endl;
    

    Eigen::Matrix4d O_T_EE;
    Eigen::VectorXd pose(7,1);

    int result = 0;
    
    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    ros::Rate loop_rate(1000);
    while (ros::ok()){


        O_T_EE = panda.O_T_EE;
        pose = panda.robot_pose(O_T_EE);
        
        result = panda.inpolygon(vertices.transpose(), pose(0), pose(1));
        std::cout << CLEANWINDOW << "Result: " << result << std::endl;
        
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            break;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}