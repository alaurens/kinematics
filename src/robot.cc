#include "robot.h"
#include <iostream>
#include "absl/types/span.h"
#include "absl/strings/str_join.h"
#include "absl/algorithm/container.h"
#include "transformations.h"
#include "utils.h"

namespace alaurens {

Robot::Robot(
    absl::string_view name,
    absl::Span<JointParameter> joint_parameters,
    absl::Span<const double> world_to_first_joint_transform,
    absl::Span<const double> last_joint_to_end_effector_transform
):name_(name), number_of_joints_(joint_parameters.size()) {

  joint_parameters_.resize(joint_parameters.size());
  dh_parameters_.resize(joint_parameters.size());
  absl::c_copy(joint_parameters, joint_parameters_.begin());

  ThrowIfSpanWrongSize(world_to_first_joint_transform, 16);
  ThrowIfSpanWrongSize(last_joint_to_end_effector_transform, 16);

  absl::c_copy(
      world_to_first_joint_transform,
      world_to_first_joint_transform_.begin()
  );

  absl::c_copy(
      last_joint_to_end_effector_transform,
      last_joint_to_end_effector_transform_.begin()
  );

}

void Robot::ForwardKinematics(absl::Span<double> result) const{
  ThrowIfSpanWrongSize(absl::Span<const double>(result), 16);

  absl::Span<const double> const_buffer(last_joint_to_end_effector_transform_);
  std::array<std::array<double, 16>, 2> buffers;
  std::array<absl::Span<double>, 2> buffer_span;
  buffer_span[0] = absl::MakeSpan(buffers[0]);
  buffer_span[1] = absl::MakeSpan(buffers[1]);

  for (auto& dh_parameter: dh_parameters_) {
    DHParametersToTransform(
        dh_parameter[0],
        dh_parameter[1],
        dh_parameter[2],
        dh_parameter[3],
        buffer_span[0]
    );
    ChainRotations(buffer_span[0], const_buffer, buffer_span[1]);
    const_buffer = buffer_span[1];
  }

  ChainRotations(world_to_first_joint_transform_, buffer_span[1], result);
  return;
}

void Robot::Jacobian(absl::Span<double> result) const {
  ThrowIfSpanWrongSize(absl::Span<const double>(result), number_of_joints_ * 6);
  return;
}

void Robot::UpdateRobotState(absl::Span<const double> joint_configuration) {
  ThrowIfSpanWrongSize(joint_configuration, number_of_joints_);
  ComputeDHParmameters(joint_configuration);
  return;
}

void ComputeDHParmameters(absl::Span<const double> joint_configuration) {
  return;
}

}  // namespace alaurens
