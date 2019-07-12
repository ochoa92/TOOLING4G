// #include "ros/ros.h"
// #include <iostream>
// #include <sstream>
// #include <string>
//
// #include <franka_spacenav/spacenav.h>
//
// int main(int argc, char** argv) {
//
//   ros::init(argc, argv, "gripper_node");
//   ros::NodeHandle nh;
//   franka_spacenav::Spacenav franka(nh);
//
//   double linear_y = 0.0;
//   double angular_z = 0.0;
//   double delta_width = 0.00001;
//   double delta_speed = 0.0001;
//
//
//   ros::Rate loop_rate(500);
//   int count = 0;
//   while (ros::ok()){
//
//     linear_y = franka.spacenav_motion(1);
//     angular_z = franka.spacenav_motion(5);
//
//     // change gripper width
//     if(linear_y > 0.0){
//       franka.width += delta_width;
//       if(franka.width > 0.08){
//         franka.width = 0.08;
//       }
//     }
//     else if(linear_y < 0.0){
//       franka.width -= delta_width;
//       if(franka.width < 0.0){
//         franka.width = 0.0;
//       }
//     }
//
//     // change gripper speed
//     if(angular_z > 0.0){
//       franka.speed += delta_speed;
//       if(franka.speed > 0.1){
//         franka.speed = 0.1;
//       }
//     }
//     else if(angular_z < 0.0){
//       franka.speed -= delta_speed;
//       if(franka.speed < 0.0){
//         franka.speed = 0.0;
//       }
//     }
//     std::cout << CLEANWINDOW << "\ngripper_width: " << franka.width << " gripper_speed: " << franka.speed << std::endl;
//
//     ros::spinOnce();
//     loop_rate.sleep();
//     count++;
//   }
//
//   return 0;
// }

#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
#include <pthread.h>

#include <franka/exception.h>
#include <franka/gripper.h>

#include <franka_spacenav/spacenav.h>

void *open_gripper(void *arg){
  franka::Gripper gripper("172.16.0.2");
  gripper.move(0.08, 0.02);
  std::cout << "Open" << std::endl;
  pthread_exit(NULL);
}

void *close_gripper(void *arg){
  franka::Gripper gripper("172.16.0.2");
  gripper.move(0.0, 0.02);
  std::cout << "Close" << std::endl;
  pthread_exit(NULL);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "gripper_test");

  ros::NodeHandle nh;
  franka_spacenav::Spacenav franka(nh);

  pthread_t thread;
  double mov = 0.0;
  int flag = 0;


  ros::Rate loop_rate(500);
  int count = 0;
  while (ros::ok()){

    mov = franka.spacenav_motion(5);
    //std::cout << mov << std::endl;

    if(mov > 0.0){
      if(flag == 0){
        pthread_create(&thread, NULL, &open_gripper, NULL);
        flag = 1;
      }
      if(flag == -1){
        pthread_cancel(thread);
        pthread_create(&thread, NULL, &open_gripper, NULL);
        flag = 1;
      }
    }
    else if(mov < 0.0){
      if(flag == 0){
        pthread_create(&thread, NULL, &close_gripper, NULL);
        flag = -1;
      }
      if(flag == 1){
        pthread_cancel(thread);
        pthread_create(&thread, NULL, &close_gripper, NULL);
        flag = -1;
      }
    }
    else{
      if(flag == 1 || flag == -1){
        pthread_cancel(thread);
        franka::Gripper gripper("172.16.0.2");
        gripper.stop();
        flag = 0;
      }
    }
    // std::cout << flag << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
}
