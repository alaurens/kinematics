#include "transformations.h"
#include "absl/strings/str_cat.h"
#include "math.h"
#include "utils.h"
#include "absl/types/span.h"
#include <Eigen/Core>

namespace alaurens {

void ApplyTransformationToVector(
  absl::Span<const double> transform, 
  absl::Span<const double> vec,
  absl::Span<double> result
) {
  ThrowIfSpanWrongSize(transform, 16);
  ThrowIfSpanWrongSize(vec, 4);
  ThrowIfSpanWrongSize(absl::Span<const double>(result), 4);
  if (vec.data() == result.data())
    throw std::invalid_argument(
      "result and vec must should point to different data."
    );

  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> transform_map(
      transform.data()
  );
  Eigen::Map<const Eigen::Vector4d> vec_map(vec.data());
  Eigen::Map<Eigen::Vector4d> result_map(result.data());

  result_map.noalias() = transform_map * vec_map;

  return;
}

void CombineTransforms(
  absl::Span<const double> transform1, 
  absl::Span<const double> transform2,
  absl::Span<double> result) {
  ThrowIfSpanWrongSize(transform1, 16);
  ThrowIfSpanWrongSize(transform2, 16);
  ThrowIfSpanWrongSize(absl::Span<const double>(result), 16);
  if (transform1.data() == result.data() || transform2.data() == result.data())
    throw std::invalid_argument(
      "result must point to different data than both transform1 and transform2."
    );

  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> transform1_map(
      transform1.data()
  );
  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> transform2_map(
      transform2.data()
  );
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> result_map(
      result.data()
  );
  result_map.noalias() = transform1_map * transform2_map;
  }

void AxisAngleToTransform(
  absl::Span<const double> axis,
  double angle,
  absl::Span<double> result
) {
  ThrowIfSpanWrongSize(axis, 3);
  ThrowIfSpanWrongSize(absl::Span<const double>(result), 16);
  if (angle < -2 * M_PI || angle > 2 * M_PI)
    throw std::invalid_argument(
      absl::StrCat(
          "The angle in the axis angle must be within [-2pi, 2pi].",
          " Received the value: ", angle
      )
    );
  Eigen::Vector3d normal(axis.data());
  normal.normalize();
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> result_map(
      result.data()
  );
  double ca = cos(angle);
  double sa = sin(angle);
  
  result_map.block<3,3>(0, 0) << 
      ca + pow(normal[0], 2) * (1 - ca),
      normal[0] * normal[1] * (1 - ca) - normal[2] * sa,
      normal[0] * normal[2] * (1 - ca) + normal[1] * sa,
      normal[0] * normal[1] * (1 - ca) + normal[2] * sa,
      ca + pow(normal[1], 2) * (1 - ca),
      normal[1] * normal[2] * (1 - ca) - normal[0] * sa,
      normal[0] * normal[2] * (1 - ca) - normal[1] * sa,
      normal[1] * normal[2] * (1 - ca) + normal[0] * sa,
      ca + pow(normal[2], 2) * (1 - ca);

  result_map(3, 3) = 1.0;
}

void DHParametersToTransform(
    double a, double alpha, double d, double phi, absl::Span<double> result
) {
  ThrowIfSpanWrongSize(absl::Span<const double>(result), 16);
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> result_map(
      result.data()
  );
  result_map << 
  cos(phi), -sin(phi), 0.0, a,
  sin(phi) * cos(alpha), cos(phi) * cos(alpha), -sin(alpha), -d * sin(alpha),
  sin(phi) * sin(alpha), cos(phi) * sin(alpha), cos(alpha), -d * cos(alpha),
  0.0, 0.0, 0.0, 1.0; 

  return;
}

}  // namespace alaurens