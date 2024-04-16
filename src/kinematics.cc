#include "robot.h"
#include "absl/types/span.h"
#include "transformations.h"
#include "utils.h"

namespace alaurens {

void ForwardKinematics(const Robot& robot,  absl::Span<double> base_to_end_effector_transform) {
  ThrowIfSpanWrongSize(
      absl::Span<const double>(base_to_end_effector_transform), 16
  );

  // Setup the buffers.
  std::array<double, 16> dh_transform;
  std::array<std::array<double, 16>, 2> buffers;
  absl::Span<double> transform_span(buffers[0]);
  int index_of_next_buffer = 1;
  // We start the computation from the base of the robot.
  absl::Span<const double> previous_transform_span(
      robot.RobotBaseToDHFirstFrame()
  );

  absl::Span<const Robot::DHParameters> dh_parameters = robot.GetDHParameters();
  // Moving up the chain of joints we apply the successive transforms to the
  // previously computed one. T01 * T12 * ... * T(n-1)n = T0n
  for (auto& dh_parameter: dh_parameters) {
    DHParametersToTransform(
        dh_parameter.a,
        dh_parameter.alpha,
        dh_parameter.d,
        dh_parameter.phi,
        absl::MakeSpan(dh_transform)
    );
    // T0(i-1) * T(i-1)i = T0i
    CombineTransforms(previous_transform_span, dh_transform, transform_span);

    previous_transform_span = transform_span;
    // We alternate the buffer for transform_span. This is needed as the matrix
    // multiplication has aliasing.
    transform_span = absl::MakeSpan(buffers[index_of_next_buffer]);
    index_of_next_buffer ^= 1;  // alternates between 0 and 1
  }

  // We apply the final transformation to get the end effector frame.
  CombineTransforms(
      previous_transform_span,
      robot.DHLastFrameToEndEffectorFrame(),
      base_to_end_effector_transform
  );
  return;
}

}  // namespace alaurens