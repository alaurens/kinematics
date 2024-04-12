#include <gtest/gtest.h>
#include "robot.h"
#include <vector>

namespace robotics {
// namespace {

// Demonstrate some basic assertions.
TEST(RobotTest, AssertThrowWhenBasePositionWrongSize) {
  std::vector<const double> links_lengths{1.0, 2.0, 3.0};
  std::vector<const double> position{0.0, 0.0, 0.0, 0.0};
  // std::unique_ptr<robotics::Robot> robot = (
  //     std::make_unique<robotics::Robot>("robot",links_lengths, position));
  ASSERT_THROW(Robot("robot",links_lengths, position), std::invalid_argument);
}
}  // namespace robotics
// }  // namespace
