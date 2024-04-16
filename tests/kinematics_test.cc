#include "kinematics.h"
#include "three_joint_robot.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "absl/types/span.h"
#include <cmath>

#include <array>

namespace alaurens {

using ::testing::Pointwise;
using ::testing::DoubleNear;

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
  std::array<double, 4> link_lengths{1.0, 1.0, 1.0, 2.0};
  std::unique_ptr<ThreeJointRobot> robot( 
      std::make_unique<ThreeJointRobot>("robot", link_lengths)
  );

  // When all the joints are 0.
  std::array<double, 16> end_effector_transform;
  ForwardKinematics(*robot, absl::MakeSpan(end_effector_transform));
  EXPECT_THAT(
      end_effector_transform,
      Pointwise(
          DoubleNear(pow(10, -5)),
          {
              0.0, 0.0, 1.0, 3.0,
              0.0, 1.0, 0.0, 0.0,
              -1.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 1.0
          }
      )
  );

  robot->UpdateRobotState({0.0, M_PI / 2, 0.0});
  ForwardKinematics(*robot, absl::MakeSpan(end_effector_transform));
  EXPECT_THAT(
      end_effector_transform,
      Pointwise(
          DoubleNear(pow(10, -5)),
          {
              1.0, 0.0, 0.0, 2.0,
              0.0, 1.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 3.0,
              0.0, 0.0, 0.0, 1.0
          }
      )
  );

  robot->UpdateRobotState({0.0, M_PI / 2, M_PI / 3});
  ForwardKinematics(*robot, absl::MakeSpan(end_effector_transform));
  EXPECT_THAT(
      end_effector_transform,
      Pointwise(
          DoubleNear(pow(10, -5)),
          {
              cos(M_PI / 3), -sin(M_PI / 3), 0.0, 2.0,
              sin(M_PI / 3), cos(M_PI / 3), 0.0, 0.0,
              0.0, 0.0, 1.0, 3.0,
              0.0, 0.0, 0.0, 1.0
          }
      )
  );

  robot->UpdateRobotState({M_PI, M_PI / 2, M_PI / 3});
  ForwardKinematics(*robot, absl::MakeSpan(end_effector_transform));
  EXPECT_THAT(
      end_effector_transform,
      Pointwise(
          DoubleNear(pow(10, -5)),
          {
              cos(4 * M_PI / 3), -sin(4 * M_PI / 3), 0.0, -2.0,
              sin(4 * M_PI / 3), cos(4 * M_PI / 3 ), 0.0, 0.0,
              0.0, 0.0, 1.0, 3.0,
              0.0, 0.0, 0.0, 1.0
          }
      )
  );

}

}  // namespace alaurens
