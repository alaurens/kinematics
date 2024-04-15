#include "kinematics.h"
#include "three_joint_robot.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "absl/types/span.h"

#include <vector>
#include <array>

namespace alaurens {

TEST(KinematicsTest, ForwardKinematicsThrowsWithWrongArguments) {
  std::array<double, 4> link_lengths{1.0, 1.0, 1.0, 1.0};
  std::unique_ptr<ThreeJointRobot> robot( 
      std::make_unique<ThreeJointRobot>("robot", link_lengths)
  );
  // Wrong length result span
  std::array<double, 3> result;
  ASSERT_THROW(
      ForwardKinematics(*robot, absl::MakeSpan(result)),
      std::invalid_argument
  );
}

TEST(KinematicsTest, ForwardKinematicsThreeJointRobotWithKnownConfigurations) {
  std::array<double, 4> link_lengths{1.0, 1.0, 1.0, 1.0};
  std::unique_ptr<ThreeJointRobot> robot( 
      std::make_unique<ThreeJointRobot>("robot", link_lengths)
  );
  
}
}  // namespace alaurens