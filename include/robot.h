#ifndef ROBOT_H
#define ROBOT_H

#include "absl/types/span.h"
#include "absl/strings/string_view.h"

namespace alaurens {

// Interface for a serial robot. Implementations of this class should pay
// attention to not allocating any memory after the object has been been
// created.
class Robot {

 public:

  // Holds one set Denavit-Hartenberg parameters.
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
  struct DHParameters {
    double a;
    double alpha;
    double d;
    double phi;
  };

  virtual absl::string_view Name() const = 0;

  // Returns the Denavit-Hartenberg parameters of the robot. As described in
  // the Appendix C of the 2019 edition of "Modern Robotics Mechanics, Planning
  // and Control." By Kevin M.Lynch and Frank C.Park.
  virtual absl::Span<const DHParameters> GetDHParameters() const = 0;

  // Returns the homoegenous transform describing the transformation from the
  // base frame of the robot to the first frame of used in the
  // Denavit-Hartenberg representation of the robot. The homoegeneous matrix 
  // is given using row major convention.
  virtual absl::Span<const double> RobotBaseToDHFirstFrame() const = 0;

  // Returns the homoegenous transform describing the transformation from the
  // the last frame of used in the Denavit-Hartenberg representation of the
  // robot to the end effector frame. The homoegeneous matrix is given using
  // row major convention.
  virtual absl::Span<const double> DHLastFrameToEndEffectorFrame() const = 0;

};

}  // namespace alaurens

#endif  // ROBOT_H
