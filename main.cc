
#include <cmath>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "robot.h"
#include <memory>
#include <utility>
 

int main(int argc, char* argv[]){
  std::vector<const double> links_lengths{1.0, 2.0, 3.0};
  std::vector<const double> position{0.0, 0.0, 0.0, 0.0};
  std::unique_ptr<robotics::Robot> robot = (
      std::make_unique<robotics::Robot>("robot",links_lengths, position));
  // robotics::Robot robot = robotics::Robot("robot",links_lengths, position);
  Eigen::Vector3f test_vec(1.0f, 3.0f, 3.0f);
  std::cout << test_vec << std::endl;
  return 0;
}