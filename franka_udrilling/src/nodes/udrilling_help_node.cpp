// =============================================================================
// Name        : udrilling_help_node.cpp
// Author      : Hélio Ochoa
// Description :
// =============================================================================
#include <franka_udrilling/spacenav.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>


// STATE MACHINE --------------------------------------------------------------
#define WAIT 0
#define PREDRILL 1
#define DRILL 2
#define ROOFUP 3
#define ROOFDOWN 4
#define MOVEUP 5
#define INTERRUPT 6
// -----------------------------------------------------------------------------

int main(int argc, char **argv){

  ros::init(argc, argv, "udrilling_help_node");
  ros::NodeHandle nh;
  franka_udrilling::Spacenav panda(nh);

  geometry_msgs::PoseStamped marker_pose;


  // ---------------------------------------------------------------------------
  // VIZUALIZATION MARKERS
  // ---------------------------------------------------------------------------
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20);
  visualization_msgs::Marker points;
  points.header.frame_id = "/panda_link0";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.id = 0;
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::POINTS;
  points.pose.orientation.w = 1.0;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.01;
  points.scale.y = 0.01;
  points.scale.z = 0.01;

  // Set the color -- be sure to set alpha to something non-zero!
  points.color.a = 1.0;
  points.color.r = 1.0;
  points.color.g = 0.0;
  points.color.b = 0.0;


  // ---------------------------------------------------------------------------
  // tf broadcaster
  // ---------------------------------------------------------------------------
  tf::TransformBroadcaster station_br, pandaEEd_br, mould_br;
  tf::Transform station_tf, pandaEEd_tf, mould_tf;


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
  // position
  Eigen::Vector3d pi, pf;
  pi << pose_i[0], pose_i[1], pose_i[2];  // define initial position
  pf << pi;
  double ti = 0.0;
  double tf = 0.0;
  double t = 0.0;
  double delta_t = 0.001;

  // orientation
  Eigen::Quaterniond oi, of;
  oi.coeffs() << pose_i[3], pose_i[4], pose_i[5], pose_i[6];
  of.coeffs() << oi.coeffs();
  Eigen::Matrix3d Rd(oi);

  geometry_msgs::Point p_mould;
  p_mould.x = pi(0);
  p_mould.y = pi(1);
  p_mould.z = pi(2);
  points.points.push_back(p_mould);

  // ---------------------------------------------------------------------------
  // DRILLING TRAJECTORY CONDITIONS
  // ---------------------------------------------------------------------------
  Eigen::Vector3d delta_drill, delta_roof, delta_predrill, delta_goal, delta_limit;
  delta_drill << 0.0, 0.0, 0.001;
  delta_roof << 0.0, 0.0, 0.001;
  delta_predrill << 0.0, 0.0, 0.003;
  delta_goal << 0.0, 0.0, 0.009;  
  delta_limit << 0.0, 0.0, 0.012; // 0.012, 0.015
  Eigen::Vector3d p_roof, p_goal, p_limit;
  p_roof.setZero();
  p_goal.setZero();
  p_limit.setZero();

  double max_force_limit = 6.0;

  // ---------------------------------------------------------------------------
  // TRAJECTORY UP CONDITIONS
  // ---------------------------------------------------------------------------
  Eigen::Vector3d delta_up;
  delta_up << 0.0, 0.0, 0.15;

  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  Eigen::Vector3d position_d(pi);
  Eigen::Quaterniond orientation_d(oi);

  int flag_drilling = 0;
  int flag_print = 0;
  int flag_interrupt = 0;
  double result = 0.0;

  // change compliance parameters
  int systemRet = 0;
  systemRet = system("rosrun dynamic_reconfigure dynparam set /dynamic_reconfigure_compliance_param_node Kpz 800.0");
  systemRet = system("rosrun dynamic_reconfigure dynparam set /dynamic_reconfigure_compliance_param_node Dpz 60.0");
  systemRet = system("rosrun dynamic_reconfigure dynparam set /dynamic_reconfigure_compliance_param_node Ipx 0.0");
  systemRet = system("rosrun dynamic_reconfigure dynparam set /dynamic_reconfigure_compliance_param_node Ipy 0.0");
  systemRet = system("rosrun dynamic_reconfigure dynparam set /dynamic_reconfigure_compliance_param_node Ipz 0.0");
  if(systemRet == -1){
    std::cout << CLEANWINDOW << "The system method failed!" << std::endl;
  }

  ros::Rate loop_rate(1000);
  int count = 0;
  while(ros::ok()){

    switch (flag_drilling) { ///////////////////////////////////////////////////

      // -----------------------------------------------------------------------
      case WAIT:
        // --> WAIT <--
        if(flag_print == 0){
          std::cout << CLEANWINDOW << "ROBOT IS READY, PLEASE PRESS BUTTON <1> OF SPACENAV TO START DRILLING!" << std::endl;
          flag_print = 1;
        }
        if(panda.spacenav_button_1 == 1){
          flag_drilling = PREDRILL;
          pi << pose_i[0], pose_i[1], pose_i[2];  // define initial position
          pf << pi + Rd*delta_predrill;
          t = 0;  // reset time
        }

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case PREDRILL:
        // --> PRE DRILL <--
        if(flag_print == 1){
          std::cout << CLEANWINDOW << "ROBOT IS PRE-DRILLING!" << std::endl;
          flag_print = 2;
        }
        ti = 0.0;
        tf = 10.0;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = DRILL;
          pi << position_d;
          pf << pi + Rd*delta_drill;
          p_roof << pi + Rd*delta_roof;
          p_goal << pi + Rd*delta_goal;
          p_limit << pi + Rd*delta_limit;
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case DRILL:
         if(flag_print == 2){
          std::cout << CLEANWINDOW << "ROBOT IS DRILLING, IF YOU WOULD LIKE TO STOP PRESS SPACENAV BUTTON <2>!" << std::endl;
          flag_print = 3;
        }

        // Force Limit -------------------------------------
        if( panda.K_F_ext_hat_K[2] > max_force_limit ){
          flag_drilling = ROOFUP;
          pi << position_d;
          pf << p_roof;
          t = 0;  // reset time
        }
        // -------------------------------------------------
        
        O_T_EE_i = panda.O_T_EE;
        pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
        result = pose_i(2) - p_goal(2);
        if( result > 0.0 ){
          // --> DRILL <--
          ti = 0.0;
          tf = 0.6;
          if( (t >= ti) && (t <= tf) ){
            position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
          }
          else if(t > tf){
            flag_drilling = ROOFUP;
            pi << position_d;
            pf << p_roof;
            t = 0;  // reset time
          }
          t = t + delta_t;
        }
        else{
          flag_drilling = MOVEUP;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          delta_up << 0.0, 0.0, 0.15;
          pf << pi - Rd*delta_up;
          t = 0;
        }

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case ROOFUP:

        // --> UP <--
        ti = 0.0;
        tf = 0.5;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = ROOFDOWN;
          pf << pi;
          pi << p_roof;
          t = 0;  // reset time
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case ROOFDOWN:

        // --> DOWN <--
        ti = 0.0;
        tf = 0.5;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = DRILL;
          pi << position_d;
          if( pi(2) < p_limit(2) ){
            pf << pi;
          }
          else{
            pf << pi + Rd*delta_drill;
          }
          t = 0;  // reset time     
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case MOVEUP:
        if(flag_print == 3){
          std::cout << CLEANWINDOW << "THE HOLE IS COMPLETE AND THE ROBOT IS MOVING UP!" << std::endl;
          flag_print = 4;
        }

        // --> MOVE UP <--
        ti = 0.0;
        tf = 3.0;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          std::cout << CLEANWINDOW << "PROGRAM FINISHED!" << std::endl;
          return(0);
        }
        t = t + delta_t;

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case INTERRUPT:

        if(flag_interrupt == 0){
          std::cout << CLEANWINDOW << "PROGRAM INTERRUPTED! IF YOU WOULD LIKE TO CONTINUE, PLEASE PRESS SPACENAV BUTTON <1>..." << std::endl;
          flag_interrupt = 1;
        }

        if(panda.spacenav_button_1 == 1){
          flag_drilling = MOVEUP;
          flag_interrupt = 0;
          flag_print = 3;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << pi - Rd*delta_up;
          t = 0;
        }

        break;

    } //////////////////////////////////////////////////////////////////////////

    // std::cout << CLEANWINDOW << position_d << std::endl;
    // std::cout << CLEANWINDOW << orientation_d.coeffs() << std::endl;
    panda.posePublisherCallback(marker_pose, position_d, orientation_d);


    // -------------------------------------------------------------------------
    // TF AND VISUALIZATION MARKERS
    // -------------------------------------------------------------------------
    // Draw the panda EE desired transform
    pandaEEd_tf.setOrigin( tf::Vector3(position_d(0), position_d(1), position_d(2)) );
    pandaEEd_tf.setRotation( tf::Quaternion(orientation_d.vec()[0], orientation_d.vec()[1], orientation_d.vec()[2], orientation_d.w()) );
    pandaEEd_br.sendTransform(tf::StampedTransform(pandaEEd_tf, ros::Time::now(), "/panda_link0", "/panda_EE_d"));

    // Draw the points
    marker_pub.publish(points);
    // -------------------------------------------------------------------------

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
}
