#include <franka_spacenav/spacenav.h>

geometry_msgs::PoseStamped marker_pose;

int main(int argc, char** argv) {

  ros::init(argc, argv, "spacenav_pattern_node");

  ros::NodeHandle nh;
  franka_spacenav::Spacenav franka(nh);


  // ---------------------------------------------------------------------------
  // GET FILE INFO
  // ---------------------------------------------------------------------------
  MatrixXd pose_d;  // matrix to save the robot poses
  ifstream infile;
  string path;
  path = "/home/panda/kst/franka/robot_patterns/pattern";
  infile.open(path);

  double p_xd, p_yd, p_zd, Q_xd, Q_yd, Q_zd, Q_wd;
  string line;
  getline(infile, line);
  int column = 0;
  pose_d.resize(7, column + 1);
  if(infile.is_open()){
    while(!infile.eof()){
      getline(infile, line);
      infile >> p_xd >> p_yd >> p_zd >> Q_xd >> Q_yd >> Q_zd >> Q_wd;
      //cout << p_xd << " " << p_yd << " " << p_zd << endl;

      // save the values in the matrix
      pose_d(0, column) = p_xd;
      pose_d(1, column) = p_yd;
      pose_d(2, column) = p_zd;
      pose_d(3, column) = Q_xd;
      pose_d(4, column) = Q_yd;
      pose_d(5, column) = Q_zd;
      pose_d(6, column) = Q_wd;
      column++;
      pose_d.conservativeResize(7, column + 1);
    }
  }
  else{
    cout << "Error open the file!" << endl;
  }
  infile.close();

  // It's necessaty eliminate the last line, because an arbitrary value appears
  column--;
  pose_d.conservativeResize(7, column);

  // ---------------------------------------------------------------------------
  // GET INITIAL POSE
  // ---------------------------------------------------------------------------
  Matrix4d O_T_EE_i;
  VectorXd pose_i(7,1);

  O_T_EE_i = franka.FK;
  pose_i = franka.robot_pose(O_T_EE_i);

  // ---------------------------------------------------------------------------
  // TRAJECTORY INITIAL CONDITIONS
  // ---------------------------------------------------------------------------
  VectorXd pi(7,1);
  VectorXd pf(7,1);
  VectorXd pd(7,1);

  pi << pose_i;
  pf << pose_d(0,0),
        pose_d(1,0),
        pose_d(2,0),
        pose_d(3,0),
        pose_d(4,0),
        pose_d(5,0),
        pose_d(6,0);
  pd << pi;

  double ti = 2.0;
  double tf = 5.0;
  double t = 0.0;


  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  Vector3d position_d;
  Quaterniond orientation_d;

  int flag_pattern = 0;

  ros::Rate loop_rate(1000);
  while (ros::ok()){

    if(flag_pattern == 0){
      cout << CLEANWINDOW << "ROBOT IS MOVING TO IS INITIAL POSE..." << endl;

      if((t >= ti) && (t <= tf)){
        pd = franka.polynomial3_trajectory(pi, pf, ti, tf, t);
      }
      else if(t > tf){
        flag_pattern = 1;
      }
      t = t + 0.001;

      position_d << pd[0],
                    pd[1],
                    pd[2];

      orientation_d.coeffs() << pd[3],
                                pd[4],
                                pd[5],
                                pd[6];

      franka.publisherCallback(marker_pose, position_d, orientation_d);

    }
    else if(flag_pattern == 1){
      cout << CLEANWINDOW << "PRESS BUTTON 1 OF THE SPACENAV TO EXECUTE PATTERN" << endl;
    }
    else if(flag_pattern == 2){
      cout << CLEANWINDOW << "PATTERN WAS EXECUTED!" << endl;

      // RESET THE INITIAL POSE AGAIN
      ros::Duration(2.0).sleep();
      O_T_EE_i = franka.FK;
      pose_i = franka.robot_pose(O_T_EE_i);
      pi << pose_i;
      pf << pose_d(0,0),
            pose_d(1,0),
            pose_d(2,0),
            pose_d(3,0),
            pose_d(4,0),
            pose_d(5,0),
            pose_d(6,0);
      pd << pi;

      t = 0;  // RESET THE TIME AGAIN

      flag_pattern = 0;
    }

    if(franka.spacenav_button_1 == 1){
      cout << CLEANWINDOW << "PATTERN IS EXECUTING, IF YOU WOULD LIKE TO STOP PRESS <S> ..." << endl;
      int count = 0;
      ros::Rate loop_rate(1000);
      while (count < column){

        position_d[0] = pose_d(0,count);
        position_d[1] = pose_d(1,count);
        position_d[2] = pose_d(2,count);
        orientation_d.vec()[0] = pose_d(3,count);
        orientation_d.vec()[1] = pose_d(4,count);
        orientation_d.vec()[2] = pose_d(5,count);
        orientation_d.w() = pose_d(6,count);

        //cout << CLEANWINDOW << position_d << endl;
        //cout << CLEANWINDOW << orientation_d.coeffs() << endl;

        franka.publisherCallback(marker_pose, position_d, orientation_d);

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
          break;

        ros::spinOnce();
        loop_rate.sleep();
        count++;
      }
      flag_pattern = 2;
    }

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
