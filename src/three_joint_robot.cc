#include "three_joint_robot.h"
#include "absl/types/span.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/algorithm/container.h"
#include "transformations.h"
#include "utils.h"
#include "math.h"

namespace alaurens {

ThreeJointRobot::ThreeJointRobot(
    absl::string_view name,
    absl::Span<const double> link_lengths
):name_(name) {
  ThrowIfSpanWrongSize(link_lengths, 4);

  // Check that none of the links are negative
  for (auto link_length: link_lengths) {
    if (link_length <= 0) {
      throw std::invalid_argument(
        absl::StrCat(
           "The links of the robot cannot be negative, was given ", link_length
        )
      );
    }
  }

  absl::c_copy(link_lengths, link_lengths_.begin());

  robot_base_frame_to_dh_first_frame_ = {
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, link_lengths[0],
      0.0, 0.0, 0.0, 1.0
  };

  dh_last_frame_to_end_effector_frame_ = {
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, link_lengths[3],
      0.0, 0.0, 0.0, 1.0
  };

  // On creation we assume all the joints of the robot are set to 0.
  dh_parameters_ = dh_parameters_ = {
      Robot::DHParameters({0.0, 0.0, 0.0, 0.0}),
      Robot::DHParameters({link_lengths[1], M_PI / 2.0, 0.0, - M_PI / 2}),
      Robot::DHParameters({link_lengths[2], - M_PI / 2.0, 0.0, 0.0})
  };

}

absl::Span<const Robot::DHParameters> 
ThreeJointRobot::GetDHParameters() const {
  return dh_parameters_;
}

absl::Span<const double> ThreeJointRobot::RobotBaseToDHFirstFrame() const {
  return robot_base_frame_to_dh_first_frame_;
}

absl::Span<const double>
ThreeJointRobot::DHLastFrameToEndEffectorFrame() const {
  return dh_last_frame_to_end_effector_frame_;
}

absl::string_view ThreeJointRobot::Name() const {
  return name_;
}

void ThreeJointRobot::UpdateRobotState(
    absl::Span<const double> joint_configuration
){
  ThrowIfSpanWrongSize(joint_configuration, 3);
  for (auto& angle: joint_configuration) {
    if (angle < -2 * M_PI || angle > 2 * M_PI) {
      throw std::invalid_argument(
        absl::StrCat(
           "The joint angles of the robot must be in the range [-2pi, 2pi].",
           " Received the value: ", angle
        )
      );
    }
  }
  dh_parameters_ = {
      Robot::DHParameters({0.0, 0.0, 0.0, joint_configuration[0]}),
      Robot::DHParameters(
          {link_lengths_[1], M_PI / 2.0, 0.0, joint_configuration[1] - M_PI / 2}
      ),
      Robot::DHParameters(
          {link_lengths_[2], - M_PI / 2.0, 0.0, joint_configuration[2]}
      )
  };
  return;
}


}  // namespace alaurens
