// =============================================================================
// Name        : universal_polishing_node.cpp
// Author      : Hélio Ochoa
// Description :
// =============================================================================
#include <franka_polishing/spacenav.h>

// STATE MACHINE --------------------------------------------------------------
#define MOVE2POINT 0
#define PATTERN 1
#define MOVEUP 2
#define INTERRUPT 3
// -----------------------------------------------------------------------------

// MAIN
int main(int argc, char** argv) {

    ros::init(argc, argv, "universal_polishing_node");

    ros::NodeHandle nh;
    franka_polishing::Spacenav panda(nh);
    geometry_msgs::PoseStamped marker_pose;


    // ---------------------------------------------------------------------------
    // GET THE CO-MANIPULATION PATTERN FROM A FILE
    // ---------------------------------------------------------------------------
    Eigen::MatrixXd position;  // matrix to save the robot positions in Base-frame
    Eigen::MatrixXd orientation;  // matrix to save the robot orientations in Base-frame
    std::ifstream pattern_file;
    std::string pattern_path;
    pattern_path = "/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/pattern";
    pattern_file.open(pattern_path);

    // t p_x p_y p_z Qx Qy Qz Qw
    double time, px, py, pz, qx, qy, qz, qw;
    std::string line;
    getline(pattern_file, line);  // first line
    getline(pattern_file, line);  // second line
    int n_pattern = 0;
    position.resize(3, n_pattern + 1);
    orientation.resize(4, n_pattern + 1);
    if(pattern_file.is_open()){
        while(pattern_file >> time >> px >> py >> pz >> qx >> qy >> qz >> qw){
            // save the values in the matrix
            position.conservativeResize(3, n_pattern + 1);
            orientation.conservativeResize(4, n_pattern + 1);
            position(0, n_pattern) = px;
            position(1, n_pattern) = py;
            position(2, n_pattern) = pz;
            orientation(0, n_pattern) = qx;
            orientation(1, n_pattern) = qy;
            orientation(2, n_pattern) = qz;
            orientation(3, n_pattern) = qw;
            n_pattern++;
        }
    }
    else{
        std::cout << "\nError open the file!" << std::endl;
        return(0);
    }
    pattern_file.close();
    //std::cout << position.transpose() << std::endl;
    //std::cout << orientation.transpose() << std::endl;

    Eigen::Quaterniond Qpattern;
    Qpattern.coeffs() = orientation.col(0);
    Qpattern.normalize();
    Eigen::Matrix3d Rpattern(Qpattern);


    // ---------------------------------------------------------------------------
    // COMPUTE OFFSETS BETWEEN POSITIONS
    // ---------------------------------------------------------------------------
    Eigen::MatrixXd OFFSET;
    OFFSET.resize(3, n_pattern-1);
    for(int i = 0; i < n_pattern-1; i++){
        OFFSET(0, i) = position(0, i+1) - position(0, i);
        OFFSET(1, i) = position(1, i+1) - position(1, i);
        OFFSET(2, i) = position(2, i+1) - position(2, i);
    }
    // std::cout << OFFSET.transpose() << std::endl;


    // ---------------------------------------------------------------------------
    // GET MOLD POINTS FROM A FILE
    // ---------------------------------------------------------------------------
    Eigen::MatrixXd Pmold;  // matrix to save the mold positions in Base-frame
    Eigen::MatrixXd Qmold;  // matrix to save the mold orientations in Base-frame
    std::ifstream mold_file;
    std::string mold_path;
    mold_path = "/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/polishing_area";
    mold_file.open(mold_path);

    // px py pz qw qx qy qz
    double Pmold_x, Pmold_y, Pmold_z, Qmold_w, Qmold_x, Qmold_y, Qmold_z;
    getline(mold_file, line);  // first line
    int n_mold = 0;
    Pmold.resize(3, n_mold + 1);
    Qmold.resize(4, n_mold + 1);
    if(mold_file.is_open()){
        while(mold_file >> Pmold_x >> Pmold_y >> Pmold_z >> Qmold_w >> Qmold_x >> Qmold_y >> Qmold_z){
            // save the values in the matrix
            Pmold.conservativeResize(3, n_mold + 1);
            Qmold.conservativeResize(4, n_mold + 1);
            Pmold(0, n_mold) = Pmold_x;
            Pmold(1, n_mold) = Pmold_y;
            Pmold(2, n_mold) = Pmold_z;
            Qmold(0, n_mold) = Qmold_x;
            Qmold(1, n_mold) = Qmold_y;
            Qmold(2, n_mold) = Qmold_z;
            Qmold(3, n_mold) = Qmold_w;
            n_mold++;
        }
    }
    else{
        std::cout << "\nError open the file!" << std::endl;
        return(0);
    }
    mold_file.close();
    // std::cout << Pmold.transpose() << std::endl;
    //std::cout << Qmold.transpose() << std::endl;


    // ---------------------------------------------------------------------------
    // DEFINE WORK-SPACE LIMITS
    // ---------------------------------------------------------------------------
    std::ifstream ws_file;
    std::string ws_path;
    double x, y, z;
    Eigen::MatrixXd P;
    ws_path = "/home/panda/catkin_ws/src/TOOLING4G/franka_polishing/co_manipulation_data/mold_workspace";
    ws_file.open(ws_path);
    int n_ws = 0;
    P.resize(n_ws + 1, 3);
    if(ws_file.is_open()){
        while(ws_file >> x >> y >> z){
            // save the values in the matrix
            P.conservativeResize(n_ws + 1, 3);
            P(n_ws, 0) = x;
            P(n_ws, 1) = y;
            P(n_ws, 2) = z;
            n_ws++;
        }
    }
    else{
        std::cout << "Error open the file!" << std::endl;
        return(0);
    }
    ws_file.close();
    std::cout << P << std::endl;

    Eigen::Vector3d P1(P.row(0));
    Eigen::Vector3d P2(P.row(1));
    Eigen::Vector3d P3(P.row(2));
    Eigen::Vector3d P4(P.row(3));

    // distances between the 4 points that delimit the plane
    Eigen::Vector3d l12(P2 - P1);
    Eigen::Vector3d l23(P3 - P2);
    Eigen::Vector3d l34(P4 - P3);
    Eigen::Vector3d l41(P1 - P4);


    // ---------------------------------------------------------------------------
    // VIZUALIZATION MARKERS
    // ---------------------------------------------------------------------------
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20);

    // ws points
    visualization_msgs::Marker points_ws;
    std::string points_ns = "points_ws";
    int points_id = 0;
    Eigen::Vector2d points_scale;
    points_scale << 0.01, 0.01;
    Eigen::Vector3d points_color;
    points_color << 1.0, 0.0, 0.0;
    points_ws = panda.pointsMarker(points_ns, points_id, points_scale, points_color);

    geometry_msgs::Point p_ws;  // work-space points
    int n = 0;  // n points
    while(n < n_ws){
        p_ws.x = P(n,0);
        p_ws.y = P(n,1);
        p_ws.z = P(n,2);
        points_ws.points.push_back(p_ws);
        n++;
    }

    // ws lines
    visualization_msgs::Marker lines_ws;
    std::string lines_ns = "lines_ws";
    int lines_id = 1;
    double lines_scale = 0.01;
    Eigen::Vector3d lines_color;
    lines_color << 1.0, 0.0, 0.0;
    lines_ws = panda.lineStripsMarker(lines_ns, lines_id, lines_scale, lines_color);
    geometry_msgs::Point l_ws;  // work-space lines
    Eigen::MatrixXd PP;
    PP.resize(5,3);
    PP << P1.transpose(), P2.transpose(), P3.transpose(), P4.transpose(), P1.transpose();
    n = 0;  // n points
    while(n < PP.rows()){
        l_ws.x = PP(n,0);
        l_ws.y = PP(n,1);
        l_ws.z = PP(n,2);
        lines_ws.points.push_back(l_ws);
        n++;
    }

    // mold points
    visualization_msgs::Marker mold_points;
    points_ns = "mold_points";
    points_id = 2;
    points_scale << 0.01, 0.01;
    points_color << 0.0, 1.0, 0.0;
    mold_points = panda.pointsMarker(points_ns, points_id, points_scale, points_color);

    geometry_msgs::Point p_m;  // mold points
    n = 0;  // n points
    while(n < n_mold){
        p_m.x = Pmold(0,n);
        p_m.y = Pmold(1,n);
        p_m.z = Pmold(2,n);
        mold_points.points.push_back(p_m);
        n++;
    }

    // polishing lines
    visualization_msgs::Marker polishing_lines;
    lines_ns = "polishing_lines";
    lines_id = 3;
    lines_scale = 0.01;
    lines_color << 0.0, 1.0, 0.0;
    polishing_lines = panda.lineStripsMarker(lines_ns, lines_id, lines_scale, lines_color);
    geometry_msgs::Point l_polishing;  // polishing lines


    // ---------------------------------------------------------------------------
    // TF BROADCASTER
    // ---------------------------------------------------------------------------
    tf::TransformBroadcaster mold_br, panda_br_d;
    tf::Transform mold_tf, panda_tf_d;


    // ---------------------------------------------------------------------------
    // GET INITIAL POSE
    // ---------------------------------------------------------------------------
    Eigen::Matrix4d O_T_EE_i;
    Eigen::VectorXd pose_i(7,1);
    O_T_EE_i = panda.O_T_EE;
    pose_i = panda.robot_pose(O_T_EE_i);


    // ---------------------------------------------------------------------------
    // TRAJECTORY TO FIRST POINT
    // ---------------------------------------------------------------------------
    Eigen::Vector3d pi, pf;
    pi << pose_i[0], pose_i[1], pose_i[2];
    pf << Pmold(0, 0), Pmold(1, 0), Pmold(2, 0);
    double ti = 1.0;
    double tf = 6.0;
    double t = 0.0;
    double delta_t = 0.001;

    // orientation
    Eigen::Quaterniond oi, of;
    oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
    // compute the desired rotation in the first point of the mold
    Eigen::Quaterniond Qmold_temp;
    Qmold_temp.coeffs() << Qmold(0, 0), Qmold(1, 0), Qmold(2, 0), Qmold(3, 0);
    Qmold_temp.normalize();
    Eigen::Matrix3d Rmould(Qmold_temp);
    Eigen::Matrix3d Rpoints(Rmould * Rpattern);
    Eigen::Quaterniond Qpoints(Rpoints);
    of.coeffs() << Qpoints.vec()[0], Qpoints.vec()[1], Qpoints.vec()[2], Qpoints.w();
    double t1 = 0.0;
    double delta_t1 = delta_t/(tf-ti);


    // ---------------------------------------------------------------------------
    // TRAJECTORY MOVE UP CONDITIONS
    // ---------------------------------------------------------------------------
    Eigen::Vector3d delta_up;
    delta_up << 0.0, 0.0, 0.1;


    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    Eigen::Vector3d position_d(pi);
    Eigen::Quaterniond orientation_d(oi);
    Eigen::Vector3d new_position_d(position_d);

    Eigen::Vector3d mould_offset;
    mould_offset.setZero();

    Eigen::Vector3d l1P, l2P, l3P, l4P;
    l1P.setZero();
    l2P.setZero();
    l3P.setZero();
    l4P.setZero();

    double signal1 = 0.0;
    double signal2 = 0.0;
    double signal3 = 0.0;
    double signal4 = 0.0;

    int flag_pattern = 0;
    int flag_print = 0;
    int flag_interrupt = 0;
    int count = 0;
    int n_points = 0;

    ros::Rate loop_rate(1000);
    while (ros::ok()){

        switch (flag_pattern) { ////////////////////////////////////////////////////

            // -----------------------------------------------------------------------
            case MOVE2POINT:
                if(flag_print == 0){
                    std::cout << CLEANWINDOW << "ROBOT IS MOVING TO A MOLD POINT..." << std::endl;
                    flag_print = 1;
                }

                // --> MOVE TO A MOLD POINT <--
                if( (t >= ti) && (t <= tf) ){
                    position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
                    if ( t1 <= 1.0 ){
                        orientation_d = oi.slerp(t1, of);
                        orientation_d.normalize();
                    }
                    t1 = t1 + delta_t1;
                }
                else if(t > tf){
                    flag_pattern = PATTERN;
                    count = 0;
                }
                t = t + delta_t;

                new_position_d = position_d;

                // INTERRUPT
                if(panda.spacenav_button_2 == 1){
                    flag_pattern = INTERRUPT;
                }

                break;

            // -----------------------------------------------------------------------
            case PATTERN:
                if(flag_print == 1){
                    std::cout << CLEANWINDOW << "PATTERN IS EXECUTING, IF YOU WOULD LIKE TO STOP PRESS SPACENAV BUTTON <2>..." << std::endl;
                    flag_print = 2;
                }

                // --> PATTERN <--
                if (count < n_pattern-1){

                    mould_offset << OFFSET(0, count), OFFSET(1, count), OFFSET(2, count);
                    new_position_d = new_position_d + Rmould * mould_offset;

                    // vectors between the pattern points and the 4 points that delimit the plane
                    l1P = (new_position_d - P1);
                    l2P = (new_position_d - P2);
                    l3P = (new_position_d - P3);
                    l4P = (new_position_d - P4);

                    // signal between the 2 vectors
                    signal1 = l1P.dot(l12);
                    signal2 = l2P.dot(l23);
                    signal3 = l3P.dot(l34);
                    signal4 = l4P.dot(l41);
                    if( (signal1 >= 0.0) && (signal2 >= 0.0) && (signal3 >= 0.0) && (signal4 >= 0.0) ){
                        position_d = new_position_d;
                    }

                }// --------------------------------------------------------------------
                else{
                    flag_pattern = MOVEUP;
                    O_T_EE_i = panda.O_T_EE;
                    pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
                    pi << pose_i[0], pose_i[1], pose_i[2];
                    pf << pi + delta_up;
                    t = 0;  // reset the time
                }
                count++;

                // draw the lines of the polished area
                if(count % 100 == 0){
                    l_polishing.x = position_d[0];
                    l_polishing.y = position_d[1];
                    l_polishing.z = position_d[2];
                    polishing_lines.points.push_back(l_polishing);
                    marker_pub.publish(polishing_lines);
                }

                // INTERRUPT
                if(panda.spacenav_button_2 == 1){
                    flag_pattern = INTERRUPT;
                }

                break;

            // -----------------------------------------------------------------------
            case MOVEUP:
                if(flag_print == 2){
                    std::cout << CLEANWINDOW << "PATTERN WAS EXECUTED AND THE ROBOT IS MOVING UP!" << std::endl;
                    flag_print = 0;
                }

                // --> MOVE UP <--
                ti = 0.0;
                tf = 3.0;
                if((t >= ti) && (t <= tf)){
                    position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
                }
                else if(t > tf){
                    if(n_points < n_mold-1){
                        n_points++; // next point        
                    }
                    else{
                        if(flag_print == 0){
                            std::cout << CLEANWINDOW << "MOULD IS ALREADY POLISHED!" << std::endl;
                            flag_print = 5;
                        }
                        return 0;
                    }

                    flag_pattern = MOVE2POINT;
                    O_T_EE_i = panda.O_T_EE;  // get current pose
                    pose_i = panda.robot_pose(O_T_EE_i);

                    // reset position
                    pi << pose_i[0], pose_i[1], pose_i[2];
                    pf << Pmold(0, n_points), Pmold(1, n_points), Pmold(2, n_points);
                    t = 0.0;  // reset t
                    ti = 0.0;
                    tf = 3.0;

                        // reset orientation
                    oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
                    // compute the desired rotation in the next point of the mold
                    Qmold_temp.coeffs() << Qmold(0, n_points), Qmold(1, n_points), Qmold(2, n_points), Qmold(3, n_points);
                    Qmold_temp.normalize();
                    Rmould = Qmold_temp;
                    Rpoints = Rmould * Rpattern;
                    Qpoints = Rpoints;
                    of.coeffs() << Qpoints.vec()[0], Qpoints.vec()[1], Qpoints.vec()[2], Qpoints.w();
                    t1 = 0.0;
                    delta_t1 = delta_t/(tf-ti);

                }
                t = t + delta_t;

                // INTERRUPT
                if(panda.spacenav_button_2 == 1){
                    flag_pattern = INTERRUPT;
                }

                break;

            // -----------------------------------------------------------------------
            case INTERRUPT:
                if(flag_interrupt == 0){
                    std::cout << CLEANWINDOW << "PROGRAM INTERRUPTED! IF YOU WOULD LIKE TO CONTINUE, PLEASE PRESS SPACENAV BUTTON <1>..." << std::endl;
                    flag_interrupt = 1;
                }

                if(panda.spacenav_button_1 == 1){
                    flag_print = 2;
                    flag_interrupt = 0;
                    flag_pattern = MOVEUP;
                    O_T_EE_i = panda.O_T_EE;
                    pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
                    pi << pose_i[0], pose_i[1], pose_i[2];
                    pf << pi + delta_up;
                    t = 0;
                }

                break;

        }//////////////////////////////////////////////////////////////////////////

        // std::cout << CLEANWINDOW << position_d << std::endl;
        // std::cout << CLEANWINDOW << orientation_d.coeffs() << std::endl;
        panda.posePublisherCallback(marker_pose, position_d, orientation_d);


        // -------------------------------------------------------------------------
        // TF AND VISUALIZATION MARKERS
        // -------------------------------------------------------------------------
        marker_pub.publish(points_ws);  // Draw the ws points
        marker_pub.publish(lines_ws); // Draw the ws lines
        marker_pub.publish(mold_points); // Draw the mold points

        // Draw the mold transform
        mold_tf.setOrigin( tf::Vector3(Pmold(0, n_points), Pmold(1, n_points), Pmold(2, n_points)) );
        mold_tf.setRotation( tf::Quaternion(Qmold(0, n_points), Qmold(1, n_points), Qmold(2, n_points), Qmold(3, n_points)) );
        mold_br.sendTransform(tf::StampedTransform(mold_tf, ros::Time::now(), "/panda_link0", "/mold"));

        // Draw the desired EE transform
        panda_tf_d.setOrigin( tf::Vector3(position_d(0), position_d(1), position_d(2)) );
        panda_tf_d.setRotation( tf::Quaternion(orientation_d.vec()[0], orientation_d.vec()[1], orientation_d.vec()[2], orientation_d.w()) );
        panda_br_d.sendTransform(tf::StampedTransform(panda_tf_d, ros::Time::now(), "/panda_link0", "/panda_EE_d"));
        // -------------------------------------------------------------------------

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
        break;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
