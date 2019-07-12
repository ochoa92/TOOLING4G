#include <franka_spacenav/spacenav.h>

geometry_msgs::PoseStamped marker_pose;

// global variables
Matrix<double, 1, 6> O_F_ext_hat_K;
Matrix<double, 1, 6> K_F_ext_hat_K;

void franka_state_callback(const franka_msgs::FrankaState::ConstPtr &msg){

  for(int i=0; i<6; i++){
    O_F_ext_hat_K(i) = msg->O_F_ext_hat_K[i];
    K_F_ext_hat_K(i) = msg->K_F_ext_hat_K[i];
  }

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "spacenav_test");

  ros::NodeHandle nh;
  franka_spacenav::Spacenav franka(nh);

  ros::Subscriber franka_state_sub; // Franka State Subscriber
  franka_state_sub = nh.subscribe("/franka_state_controller/franka_states", 20, &franka_state_callback);

  ros::Duration(1.0).sleep();

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
  VectorXd offset(7,1);

  pi << pose_i;
  //offset << 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0;
  offset << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  pf << pi + offset;
  pd << pi;

  double ti =  2.0;
  double tf = 5.0;
  double t = 0.0;

  // ellipse initial conditions
  Vector3d position_i;
  position_i.setZero();
  double t_ellipse = 0.0;
  double tf_ellipse = 60.0;
  double a = 0.1; // radio
  double b = 0.1; // radio
  double T = 10.0;  // ellipse amplitude
  double Wr = (2*PI)/T; // Ressonance frequency

  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  Vector3d position_d;
  Quaterniond orientation_d;
  int flag = 0;

  ros::Rate loop_rate(1000);
  int count = 0;
  while (ros::ok()){

    if(flag == 0){
      //cout << CLEANWINDOW << "O_F_ext_hat_K: " << O_F_ext_hat_K << endl;
      cout << CLEANWINDOW << "K_F_ext_hat_K(N): " << K_F_ext_hat_K << endl << endl;
    }

    if(K_F_ext_hat_K[2] < -5.0){
      flag = 1;
    }

    if(flag == 1){
      cout << CLEANWINDOW << "t(s): " << t << ";" << " t_ellipse: " << t_ellipse << ";" << " pose_d(m): " << pd.transpose() << endl;

      if((t >= ti) && (t <= tf)){
        pd = franka.polynomial3_trajectory(pi, pf, ti, tf, t);
        position_i << pd[0],
                      pd[1],
                      pd[2];
      }
      else if(t > tf){
        if(t_ellipse <= tf_ellipse){
          pd[0] = position_i[0] + 0.0;
          pd[1] = position_i[1] + a * cos(Wr * t_ellipse) - a;
          pd[2] = position_i[2] + b * sin(Wr * t_ellipse);
        }
        t_ellipse = t_ellipse + 0.005;
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

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
}
