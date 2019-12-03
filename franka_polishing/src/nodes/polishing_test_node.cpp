// =============================================================================
// Name        : polishing_test_node.cpp
// Author      : Hélio Ochoa
// Description :
// =============================================================================
#include <franka_polishing/spacenav.h>

// This uses the ray-casting algorithm to decide whether the point is inside
int inpolygon(const Eigen::MatrixXd &poly, double x, double y)
{
    // If we never cross any lines we're inside.
    int inside = 0;

    // Loop through all the edges.
    for (int i = 0; i < poly.rows(); ++i)
    {
        // i is the index of the first vertex, j is the next one.
        int j = (i + 1) % poly.rows();

        // The vertices of the edge we are checking.
        double xp0 = poly(i, 0);
        double yp0 = poly(i, 1);
        double xp1 = poly(j, 0);
        double yp1 = poly(j, 1);

        // Check whether the edge intersects a line from (-inf,y) to (x,y).
        
        // First check if the line crosses the horizontal line at y in either direction.
        if ( ((yp0 <= y) && (yp1 > y)) || ((yp1 <= y) && (yp0 > y)) ){
            // If so, get the point where it crosses that line. This is a simple solution
            // to a linear equation. Note that we can't get a division by zero here -
            // if yp1 == yp0 then the above if be false.
            double cross = (xp1 - xp0) * (y - yp0) / (yp1 - yp0) + xp0;

            // Finally check if it crosses to the left of our test point. You could equally
            // do right and it should give the same result.
            if (cross < x){
                inside = !inside;
            }
        }
    }
    return inside;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "polishing_test_node");

    ros::NodeHandle nh;
    franka_polishing::Spacenav panda(nh);
    geometry_msgs::PoseStamped marker_pose;

    // ---------------------------------------------------------------------------
    // GET POLYGON VERTICES FROM A FILE
    // ---------------------------------------------------------------------------
    Eigen::MatrixXd polygon_vertices;  // matrix to save the polygon_vertices
    std::ifstream polygon_file;
    polygon_file.open("/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/polygon_vertices");

    // Vx Vy
    double Vx, Vy;
    std::string line;
    getline(polygon_file, line);  // first line
    int n_vertices = 0;
    polygon_vertices.resize(2, n_vertices + 1);
    if(polygon_file.is_open()){
        while(polygon_file >> Vx >> Vy){
            // save the values in the matrix
            polygon_vertices.conservativeResize(2, n_vertices + 1);
            polygon_vertices(0, n_vertices) = Vx;
            polygon_vertices(1, n_vertices) = Vy;
            n_vertices++;
        }
    }
    else{
        std::cout << "\nError open the file!" << std::endl;
        return(0);
    }
    polygon_file.close();
    // std::cout << polygon_vertices.transpose() << std::endl;

    int inside;
    // double x = 0.5059;
    // double y = 0.3174;   
    // inside = inpolygon(polygon_vertices.transpose(), x, y);
    // std::cout << "\nResult: " << inside << std::endl;

    Eigen::Matrix4d O_T_EE;
    Eigen::VectorXd pose(7,1);

    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    ros::Rate loop_rate(1000);
    while (ros::ok()){


        O_T_EE = panda.O_T_EE;
        pose = panda.robot_pose(O_T_EE);
        inside = inpolygon(polygon_vertices.transpose(), pose(0), pose(1));
        std::cout << CLEANWINDOW << "Result: " << inside << std::endl;

        
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
        break;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}