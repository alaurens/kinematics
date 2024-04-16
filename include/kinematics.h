#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "robot.h"
#include "absl/types/span.h"

namespace alaurens {

// Computes the transform between the base of the robot and the end effector.
//
// Will output the homogeneous transform from the base of the robot to the end 
// effector which corresponds to the position and the orientation of the end
// effector in the base frame of the robot.
//
// Arguments:
// - robot: The robot for which we want to compute the forward kinematics.
//   We assume that the denavit-hartenberg parameters returns by the robot are
//   have been computed based on the joint configuration we are computing the
//   forward kinematics for.
// - result: A buffer that we be filled with the final transfor. We assume it
//   has a size of 16.
//
// Throws:
// - If `result.size != 16`.
void ForwardKinematics(
    const Robot& robot, absl::Span<double> base_to_end_effector_transform
);

}

#endif  // KINEMATICS_H
