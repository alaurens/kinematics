#include "transformations.h"
#include <Eigen/Core>
#include <iostream>
#include "absl/strings/str_cat.h"
#include "math.h"
#include "utils.h"
#include "absl/types/span.h"

namespace alaurens {


void RotateVector(
  absl::Span<const double> rotation_matrix, 
  absl::Span<const double> vector_to_rotate,
  absl::Span<double> result
) {
  ThrowIfSpanWrongSize(rotation_matrix, 16);
  ThrowIfSpanWrongSize(vector_to_rotate, 4);
  ThrowIfSpanWrongSize(absl::Span<const double>(result), 4);

  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> rot_mat(
      rotation_matrix.data()
  );
  Eigen::Map<const Eigen::Vector4d> vec(vector_to_rotate.data());
  Eigen::Map<Eigen::Vector4d> res(result.data());

  res.noalias() = rot_mat * vec;

  return;
}

void ChainRotations(
  absl::Span<const double> rotation_matrix1, 
  absl::Span<const double> rotation_matrix2,
  absl::Span<double> result) {
  ThrowIfSpanWrongSize(rotation_matrix1, 16);
  ThrowIfSpanWrongSize(rotation_matrix2, 16);
  ThrowIfSpanWrongSize(absl::Span<const double>(result), 16);

  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> rot_mat1(
      rotation_matrix1.data()
  );
  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> rot_mat2(
      rotation_matrix2.data()
  );
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> res(
      result.data()
  );
  res.noalias() = rot_mat1 * rot_mat2;
  }

void DHParametersToTransform(
    double a, double alpha, double d,  double theta, absl::Span<double> result
) {
  ThrowIfSpanWrongSize(absl::Span<const double>(result), 16);
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> res(
      result.data()
  );

  res << 
  cos(theta), -sin(theta), 0, a,
  sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha),
  sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), -d * cos(alpha),
  0, 0, 0, 1; 

  return;
}

}  // namespace alaurens