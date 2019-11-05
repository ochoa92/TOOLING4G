// =============================================================================
// Name        : test_node.cpp
// Author      : Hélio Ochoa
// Description :             
// =============================================================================
#include <franka_udrilling/udrilling_state.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <iostream>



int main(int argc, char **argv){

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    franka_udrilling::uDrillingState panda(nh);

    
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "/panda_link0";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.id = 0;
    line_strip.id = 1;
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;


    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.01;
    points.scale.y = 0.01;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    points.color.a = 1.0;
    points.color.r = 1.0f;
    points.color.g = 0.0f;
    points.color.b = 0.0f;

    // Line strip is blue
    line_strip.color.a = 1.0;
    line_strip.color.b = 1.0;

    // ---------------------------------------------------------------------------
    // GET INITIAL POSE
    // ---------------------------------------------------------------------------
    Eigen::Matrix4d O_T_EE;
    Eigen::VectorXd pose(7,1);


    // ---------------------------------------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------------------------------------
    ros::Rate loop_rate(100);
    int count = 0;
    while (ros::ok()){

        O_T_EE = panda.O_T_EE;
        pose = panda.robotPose(O_T_EE);

        geometry_msgs::Point p;
        p.x = pose[0];
        p.y = pose[1];
        p.z = pose[2];

        points.points.push_back(p);
        line_strip.points.push_back(p);

        // if(line_strip.points.size() > 30){
        //   line_strip.points.erase(line_strip.points.begin());
        // }
        //
        // if(points.points.size() > 30){
        //   points.points.erase(points.points.begin());
        // }
        
        marker_pub.publish(points);
        // marker_pub.publish(line_strip);

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
        break;

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
   

    return 0;
}


