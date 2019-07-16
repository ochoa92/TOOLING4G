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
#define DRILL 1
#define ROOFUP 2
#define ROOFDOWN 3
#define MOVEUP 4
#define INTERRUPT 5
// -----------------------------------------------------------------------------

int main(int argc, char **argv){

  ros::init(argc, argv, "udrilling_help_node");
  ros::NodeHandle nh;
  franka_udrilling::Spacenav panda(nh);

  geometry_msgs::PoseStamped marker_pose;


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


  // ---------------------------------------------------------------------------
  // DRILLING TRAJECTORY CONDITIONS
  // ---------------------------------------------------------------------------
  Eigen::Vector3d delta_drill, delta_roof, delta_goal, delta_limit;
  delta_drill << 0.0, 0.0, 0.002;
  delta_roof << 0.0, 0.0, 0.002;
  delta_goal << 0.0, 0.0, 0.012;
  delta_limit << 0.0, 0.0, 0.015;
  Eigen::Vector3d p_roof, p_goal, p_limit;
  p_roof.setZero();
  p_goal.setZero();
  p_limit.setZero();

  double max_force_limit = 15.0;
  double min_force_limit = -2.0;

  // ---------------------------------------------------------------------------
  // TRAJECTORY UP CONDITIONS
  // ---------------------------------------------------------------------------
  Eigen::Vector3d delta_up;
  delta_up << 0.0, 0.0, 0.1;

  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  Eigen::Vector3d position_d(pi);
  Eigen::Quaterniond orientation_d(oi);

  int flag_drilling = 0;
  int flag_print = 0;
  int flag_interrupt = 0;
  double result = 0.0;

  ros::Rate loop_rate(1000);
  int count = 0;
  while(ros::ok()){

    switch (flag_drilling) { ///////////////////////////////////////////////////

      // -----------------------------------------------------------------------
      case WAIT:

        if(flag_print == 0){
          std::cout << CLEANWINDOW << "ROBOT IS READY, PLEASE PRESS BUTTON <1> OF SPACENAV TO START DRILLING!" << std::endl;
          flag_print = 1;
        }
        if(panda.spacenav_button_1 == 1){
          flag_drilling = DRILL;
          pi << pose_i[0], pose_i[1], pose_i[2];  // define initial position
          pf << pi + Rd*delta_drill;
          p_roof << pi + Rd*delta_roof;
          p_goal << pi + Rd*delta_goal;
          p_limit << pi + Rd*delta_limit;
          t = 0;  // reset time
        }

        // INTERRUPT
        if(panda.spacenav_button_2 == 1){
          flag_drilling = INTERRUPT;
        }

        break;

      // -----------------------------------------------------------------------
      case DRILL:
        if(flag_print == 1){
          std::cout << CLEANWINDOW << "ROBOT IS DRILLING, IF YOU WOULD LIKE TO STOP PRESS SPACENAV BUTTON <2>..."  << std::endl;
          flag_print = 2;
        }

        O_T_EE_i = panda.O_T_EE;
        pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
        result = pose_i(2) - p_goal(2);
        if( result > 0.0 || panda.K_F_ext_hat_K[2] > min_force_limit){
          // --> DRILL <--
          ti = 0.0;
          tf = 1.5;
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
          // flag_print = 4;
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
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
        tf = 0.8;
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
        tf = 0.8;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){
          flag_drilling = DRILL;
          pi << position_d;
          if( pi(2) < p_limit(2) || panda.K_F_ext_hat_K[2] > max_force_limit ){
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
        if(flag_print == 2){
          std::cout << CLEANWINDOW << "THE HOLE IS COMPLETE AND THE ROBOT IS MOVING UP!" << " result(m): " << result << std::endl;
          flag_print = 3;
        }

        // --> MOVE UP <--
        ti = 0.0;
        tf = 3.0;
        if( (t >= ti) && (t <= tf) ){
          position_d = panda.polynomial3_trajectory(pi, pf, ti, tf, t);
        }
        else if(t > tf){

          std::cout << CLEANWINDOW << "PROGRAM FINISHED!" << std::endl;
          break;

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
          O_T_EE_i = panda.O_T_EE;
          pose_i = panda.robot_pose(O_T_EE_i);  // get current pose
          pi << pose_i[0], pose_i[1], pose_i[2];
          pf << pi - Rd*delta_up;
          t = 0;
        }

        break;

    } //////////////////////////////////////////////////////////////////////////

    // //Define Z limit
    // if(panda.spacenav_motion(2) > 0.0){
    //   p_limit(2) += limit_z;
    // }
    // else if(panda.spacenav_motion(2) < 0.0){
    //   p_limit(2) -= limit_z;
    // }

    // std::cout << CLEANWINDOW << position_d << std::endl;
    // std::cout << CLEANWINDOW << orientation_d.coeffs() << std::endl;
    panda.posePublisherCallback(marker_pose, position_d, orientation_d);

    // -------------------------------------------------------------------------

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
}
