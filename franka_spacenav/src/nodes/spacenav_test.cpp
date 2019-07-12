#include <franka_spacenav/spacenav.h>

// global variables
Eigen::Matrix4d F_T_EE;


void franka_state_callback(const franka_msgs::FrankaState::ConstPtr &msg){

  int k = 0;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      F_T_EE(j,i) = msg->F_T_EE[k];
      k++;
    }
  }

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "polishing_test");

  ros::NodeHandle nh;
  ros::Subscriber franka_state_sub; // Franka State Subscriber
  franka_state_sub = nh.subscribe("/franka_state_controller/franka_states", 20, &franka_state_callback);


  // ---------------------------------------------------------------------------
  // MAIN LOOP
  // ---------------------------------------------------------------------------
  ros::Rate loop_rate(30);
  int count = 0;
  while (ros::ok()){

    cout << F_T_EE << endl;

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
      break;

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
}
