#include "three_joint_robot.h"
#include <gtest/gtest.h>
#include "gmock/gmock.h"
#include "absl/types/span.h"
#include "math.h"
#include <array>

namespace alaurens {

TEST(ThreeJointRobotTest, AssertThrowWithInvalidLinkSize) {
  // Test with negative link size.
  ASSERT_THROW(
      ThreeJointRobot("robot", {1.0, 2.0, 3.0, -4.0}),
      std::invalid_argument
  );

  // Test with array of wrong size.
  ASSERT_THROW(
      ThreeJointRobot("robot", {1.0, 2.0, 3.0}),
      std::invalid_argument
  );
}

TEST(ThreeJointRobotTest, RobotBaseToDHFirstFrame) {
  ThreeJointRobot robot("robot", {1.0, 2.0, 3.0, 4.0});
  EXPECT_THAT(
    robot.RobotBaseToDHFirstFrame(),
    ::testing::Pointwise(
        ::testing::DoubleEq(), std::array<double, 16>{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 1.0,
        0.0, 0.0, 0.0, 1.0}
    ))
  ;
}

TEST(ThreeJointRobotTest, DHLastFrameToEndeffectorFrame) {
  ThreeJointRobot robot("robot", {1.0, 2.0, 3.0, 4.0});
  EXPECT_THAT(
    robot.DHLastFrameToEndEffectorFrame(),
    ::testing::Pointwise(
        ::testing::DoubleEq(), std::array<double, 16>{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 4.0,
        0.0, 0.0, 0.0, 1.0}
    )
  );
}

TEST(ThreeJointRobotTest, CanUpdateJointConfiguration) {
  ThreeJointRobot robot("robot", {1.0, 2.0, 3.0, 4.0});
  robot.UpdateRobotState({0.0, 2.0, 1.0});
}

TEST(ThreeJointRobotTest, UpdateJointConfigurationWithWrongAnglesThrows) {
  ThreeJointRobot robot("robot", {1.0, 2.0, 3.0, 4.0});
  // Test with an angle larger than 2pi.
  ASSERT_THROW(
      robot.UpdateRobotState({0.0, 2.0, 100.0}), std::invalid_argument
  );

  // Test with an angle smaller than -2pi.
  ASSERT_THROW(
      robot.UpdateRobotState({0.0, -100.0, 1.0}), std::invalid_argument
  );

  // Test with the wrong number of joint angles.
  ASSERT_THROW(
      robot.UpdateRobotState({0.0, -2.0}), std::invalid_argument
  );
}

TEST(ThreeJointRobotTest, DHParametersCorrectOnStart) {
  ThreeJointRobot robot("robot", {1.0, 2.0, 3.0, 4.0});
  std::array<Robot::DHParameters, 3> expected_dh_parameters({
      Robot::DHParameters({0.0, 0.0, 0.0, 0.0}),
      Robot::DHParameters({2.0, M_PI / 2.0, 0.0, - M_PI / 2}),
      Robot::DHParameters({3.0, - M_PI / 2.0, 0.0, 0.0})
  });
  
  auto robot_dh_parameters = robot.GetDHParameters();
  for (int i = 0; i < 3; i++){
    EXPECT_EQ(robot_dh_parameters[i].a, expected_dh_parameters[i].a);
    EXPECT_EQ(robot_dh_parameters[i].alpha, expected_dh_parameters[i].alpha);
    EXPECT_EQ(robot_dh_parameters[i].d, expected_dh_parameters[i].d);
    EXPECT_EQ(robot_dh_parameters[i].phi, expected_dh_parameters[i].phi);
  }
}

TEST(ThreeJointRobotTest, DHParametersUpdateAfterRobotStateUpdate) {
  ThreeJointRobot robot("robot", {1.0, 2.0, 3.0, 4.0});

  // Update the joint configuration and check that the values of the DH
  // parameters are correct.
  robot.UpdateRobotState({1.0, M_PI, 0.0});
  std::array<Robot::DHParameters, 3> expected_dh_parameters({
      Robot::DHParameters({0.0, 0.0, 0.0, 1.0}),
      Robot::DHParameters({2.0, M_PI / 2.0, 0.0, M_PI / 2}),
      Robot::DHParameters({3.0, - M_PI / 2.0, 0.0, 0.0})
  });
  auto robot_dh_parameters = robot.GetDHParameters();
  for (int i = 0; i < 3; i++){
    EXPECT_EQ(robot_dh_parameters[i].a, expected_dh_parameters[i].a);
    EXPECT_EQ(robot_dh_parameters[i].alpha, expected_dh_parameters[i].alpha);
    EXPECT_EQ(robot_dh_parameters[i].d, expected_dh_parameters[i].d);
    EXPECT_EQ(robot_dh_parameters[i].phi, expected_dh_parameters[i].phi);
  }

  // Update the joint configuration and check that the values of the DH
  // parameters are correct.
  robot.UpdateRobotState({1.5, 0.3, -2.0});
  expected_dh_parameters = {
      Robot::DHParameters({0.0, 0.0, 0.0, 1.5}),
      Robot::DHParameters({2.0, M_PI / 2.0, 0.0, 0.3 - M_PI / 2}),
      Robot::DHParameters({3.0, - M_PI / 2.0, 0.0, -2.0})
  };
  
  robot_dh_parameters = robot.GetDHParameters();
  for (int i = 0; i < 3; i++){
    EXPECT_EQ(robot_dh_parameters[i].a, expected_dh_parameters[i].a);
    EXPECT_EQ(robot_dh_parameters[i].alpha, expected_dh_parameters[i].alpha);
    EXPECT_EQ(robot_dh_parameters[i].d, expected_dh_parameters[i].d);
    EXPECT_EQ(robot_dh_parameters[i].phi, expected_dh_parameters[i].phi);
  }
}

}  // namespace alaurens
