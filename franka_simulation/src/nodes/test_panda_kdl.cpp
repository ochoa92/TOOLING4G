#include <franka_simulation/panda_kdl.h>

using namespace P_KDL;

// MAIN
int main(int argc, char **argv){
  ros::init(argc, argv, "test_panda_kdl");

  ros::NodeHandle nh;
  P_KDL::panda_kdl kin = panda_kdl(nh);
  KDL::Chain chain = kin.kdl_chain;
  // std::cout<<"num of joints is "<<chain.getNrOfJoints()<<std::endl;
  // std::cout<<"num of links is "<<kin.kdl_chain.getNrOfSegments()<<std::endl;
  int num_joints = chain.getNrOfJoints();   // number of joints


  // ============================ FK ============================
  KDL::Frame fk_result;
  printf("\npanda position FK:\n");
  kin.FK(fk_result, kin.joint_position);
  for(int i=0; i<4; i++){
    for(int j=0; j<4; j++)
      printf("%lf, ", fk_result(i,j));
    printf("\n");
  }
  printf("\n");

  // ============================ JACOBIAN ============================
  KDL::Jacobian J;
  printf("\npanda Jacobian:\n");
  kin.jacobian(J, kin.joint_position);
  for(int i=0; i<6; i++){
    for(int j=0; j<num_joints; j++)
      printf("%lf, ", J(i,j));
    printf("\n");
  }
  printf("\n");

  // ============================ Dynamic ============================
  KDL::JntSpaceInertiaMatrix M;
  KDL::JntArray C;
  KDL::JntArray g;
  Vector gravity = {0.0, 0.0, -9.80665};   // Gravity vector
  kin.dynamic(M, C, g, kin.joint_position, kin.joint_velocity, gravity);
  printf("\npanda Inertia Matrix:\n");
  for(int i=0; i<num_joints; i++){
    for(int j=0; j<num_joints; j++)
      printf("%lf, ", M(i,j));
    printf("\n");
  }
  printf("\n");

  printf("\npanda Coriolis Vector:\n");
  for(int i=0; i<num_joints; i++){
    printf("%lf, ", C(i));
  }
  printf("\n");

  printf("\npanda Gravity Vector:\n");
  for(int i=0; i<num_joints; i++){
    printf("%lf, ", g(i));
  }
  printf("\n");

  return 0;
}
