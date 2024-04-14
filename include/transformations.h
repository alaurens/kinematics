#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include "absl/types/span.h"

namespace alaurens {

void RotateVector(
  absl::Span<const double> rotation_matrix, 
  absl::Span<const double> vec,
  absl::Span<double> result);

void ChainRotations(
  absl::Span<const double> rotation_matrix1, 
  absl::Span<const double> rotation_matrix2,
  absl::Span<double> result);

void DHParametersToTransform(
    double a, double alpha, double d, double theta, absl::Span<double> result
);

}  // namespace alaurens
#endif  // TRANSFORMATIONS_H