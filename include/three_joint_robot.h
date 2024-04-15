#include "robot.h"
#include "absl/types/span.h"
#include "absl/strings/string_view.h"
#include <vector>
#include <array>

namespace alaurens {

// Implementation of the 3R robot from Fig 4.3 of the 2019 edition of "Modern
// Robotics Mechanics, Planning and Control." By Kevin M.Lynch and Frank C.Park.
//
// Important mentions:
// - Before calling the method `GetDhParameters` the user must update the
//   joint configuration of the robot via the method `UpdateRobotState`. 
// - For simplicity we assume that the robot has no joint limits and every
//   joint can be set to any value in [0, 2pi].
// - This object is not multi-thread safe.
class ThreeJointRobot: public Robot {

 public:

  // Constructor.
  //
  // Arguments:
  // - name: Name of the robot.
  // - link_lengths: Lengths of the 4 links of the robot. For more information
  //   on how the links are setup please refer to Fig 4.3 of the 2019 edition
  //   of "ModernRobotics Mechanics, Planning and Control." By Kevin M.Lynch
  //   and Frank C.Park.
  //
  // Throws:
  // - If link_lengths.size != 4.
  // - If any of the links has a negative length.
  explicit ThreeJointRobot(
    absl::string_view name, absl::Span<const double> link_lengths
  );

  ThreeJointRobot(const ThreeJointRobot&) = delete;
  ThreeJointRobot& operator=(const ThreeJointRobot&) = delete;

  // Returns the DHParameters of the robot.
  //
  // This will update after a call to `UpdateRobotState`. Note that after
  // construction, we assume that all the joints of the robot are set to 0.
  absl::Span<const DHParameters> GetDHParameters() const override;

  absl::Span<const double> RobotBaseToDHFirstFrame() const override;

  absl::Span<const double> DHLastFrameToEndEffectorFrame() const override;

  absl::string_view Name() const override;

  // Updates the joint configuration of the robot to the provided one.
  // 
  // Note: This should be called before calling the `GetDhParameter` to ensure
  // correct results.
  // 
  // Arguments:
  // - joint_configuration: The joint angles in radians of the robot. Starting
  //   from the joint closest to the base.
  // 
  // Throws:
  // - If any of the values are not in the range [-2pi, 2pi].
  // - If the lengths of the span is not equal to the number of joints (3).
  void UpdateRobotState(absl::Span<const double> joint_configuration);

 private:
  std::string name_;
  std::array<double, 4> link_lengths_;
  std::array<DHParameters, 3> dh_parameters_;
  std::array<double, 16> robot_base_frame_to_dh_first_frame_;
  std::array<double, 16> dh_last_frame_to_end_effector_frame_;

};

}  // namespace alaurens
