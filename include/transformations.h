#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include "absl/types/span.h"

namespace alaurens {

// Applies the homogeneous transform to the vector.
//
// We assume the both the transform and the vector are expressed in the same
// base.
//
// Arguments:
// - transform: The homoegenous transformation to apply to the vector.
//   We assume that the data is arranged in row major. 
// - vec: A 4D homogeneous representation vector in the form [x, y, z, 1].
// - result: A buffer of size 4 in which the result will be written. Note that 
//   this span must be different than `vec`.
//
// Throws:
// - If `transform.size != 16`.
// - If `vec.size != 4`.
// - If `result.size != 4`.
// - If `vec` and `result` point to the same span. 
void ApplyTransformationToVector(
  absl::Span<const double> transform, 
  absl::Span<const double> vec,
  absl::Span<double> result
);

// Chains 2 transforms together.
//
// This can be used to combine 2 transforms into a single one. Assume we have 
// `H_ab` and `H_bc` the transforms that bring a to b and b to c respectively, 
// we can compute `H_ac = ChainTransforms(H_ab, H_bc)` the transform that brings
// a to c.
//
// Arguments:
// - transform1: The first transform expressed as a homoegenous matrix of size
//   4x4 with data arranged in row major.
// - transform2: The second transform expressed as a homoegenous matrix of size
//   4x4 with data arranged in row major. 
// - result: A size 16 buffer in which we will place the result. The data will
//   be arranged in row major. Note that this span must not different than both
//   transform1 and transform2. 
//
// Throws:
// - If `transform1.size != 16`.
// - If `transform2.size != 16`.
// - If `result.size != 16`.
// - If `transform1` and `result` point to the same span. 
// - If `transform2` and `result` point to the same span. 
void CombineTransforms(
  absl::Span<const double> transform1, 
  absl::Span<const double> transform2,
  absl::Span<double> result
);

// Computes the rotation matrix equivalent to the axis angle provided.
//
// Arguments:
// - axis: A 3D vector representing the axis of rotation.
// - angle: In radians, the angle of rotation. Assume that will be in
//   [-2pi, 2pi].
// - result: A size 16 buffer in which we will place the result. The data will
//   be arranged in row major.
//
// Throws:
// - If `axis.size != 16`.
// - If `angle < -2pi || angle > 2pi 
// - If `result.size != 16`.
void AxisAngleToTransform(
  absl::Span<const double> axis,
  double angle,
  absl::Span<double> result
);

// Compute the homogenous transform of a set of Denavit-Hartenberg parameters.
//
// The naming of the parameters is based on  Appendix C of the 2019 edition
// of "Modern Robotics Mechanics, Planning and Control." By Kevin M.Lynch
// and Frank C.Park. We assume that angles are given in radians and distances
// in meters.
//
// Attributes:
// - a: In meters, the length of the mutually perpendicular line between the
//   axis z_(i-1) and z_i along the direction x_(i-1).
// - alpha: The angle in radians from z_(i-1) to z_i measured about x_(i-1)
// - d: The distance from the intersection of x_(i-1) and z_i to the origin of
//   the ith frame along the z_i axis.
// - phi: The angle in radians from x_(i-1) to x_i measured along the z_i 
//   axis.
// - result: A buffer array in which we will place the resulting homogeneous
//   transform. The matrix will be arranged in row major.
//
// Throws:
// - If `result.size != 16`.
void DHParametersToTransform(
    double a, double alpha, double d, double phi, absl::Span<double> result
);

}  // namespace alaurens
#endif  // TRANSFORMATIONS_H