#ifndef ROBOT_H
#define ROBOT_H

#include "absl/types/span.h"
#include "absl/strings/string_view.h"
#include <array>

namespace alaurens {

class Robot {
 public:
  struct JointParameter {
    // Distance in m of the frame of joint i from the frame of joint i-1 frame
    // along the x axis of joint i-1.
    double link_length;
    // Rotation matrix describing the orientation frame of joint i in the 
    // frame joint of joint i-1 when joint i is at 0.
    std::array<double, 9> orientation_rotation_frame;
  };

  explicit Robot(
    absl::string_view name,
    absl::Span<JointParameter> joint_parameters,
    absl::Span<const double> world_to_first_joint_transform,
    absl::Span<const double> last_joint_to_end_effector_transform
  );

  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;

  void ForwardKinematics(absl::Span<double> result) const; 

  void Jacobian(absl::Span<double> result) const;

  void UpdateRobotState(absl::Span<const double> joint_configuration);

 private:

  void ComputeDHParmameters(absl::Span<const double> joint_configuration);

  std::string name_;
  int number_of_joints_;
  std::vector<JointParameter> joint_parameters_;
  std::vector<std::array<double, 4>> dh_parameters_;
  std::array<double, 16> world_to_first_joint_transform_;
  std::array<double, 16> last_joint_to_end_effector_transform_;

};

}  // namespace alaurens

#endif  // ROBOT_H
