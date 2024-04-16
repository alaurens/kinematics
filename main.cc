#include <iostream>
#include <cmath>
#include <string>
#include "kinematics.h"
#include "three_joint_robot.h"
#include "absl/strings/str_join.h"
#include <memory>
#include <utility>
#include <array>
  

int main(int argc, char* argv[]){

  // Change the link lengths of the robot here.
  std::array<double, 4> link_lengths{1.0, 1.0, 1.0, 1.0};
  std::unique_ptr<alaurens::ThreeJointRobot> robot( 
      std::make_unique<alaurens::ThreeJointRobot>("robot", link_lengths)
  );

  // Change the joint configuration of the robot here.
  robot->UpdateRobotState({M_PI / 2, 0.0, 0.0});
  std::array<double, 16> result;
  alaurens::ForwardKinematics(*robot, absl::MakeSpan(result));
  for (int i=0; i < 4; i++){
   std::cout 
       <<  absl::StrJoin(absl::MakeSpan(result).subspan(i*4, 4), ", ")
       << std::endl; 
  }
  return 0;
}