#include <gtest/gtest.h>
#include "robot.h"
#include <vector>

namespace alaurens {

// Demonstrate some basic assertions.
TEST(RobotTest, AssertThrowWhenBasePositionWrongSize) {
  std::vector<const double> links_lengths{1.0, 2.0, 3.0};
  std::vector<const double> position{0.0, 0.0, 0.0, 0.0};
  // ASSERT_THROW(Robot("robot",links_lengths, position), std::invalid_argument);
}

}  // namespace alaurens
