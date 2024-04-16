#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "robot.h"
#include "absl/types/span.h"

namespace alaurens {

// Computes the transform between the base of the robot and the end effector.
//
// Will output 
void ForwardKinematics(const Robot& robot, absl::Span<double> result);

}

#endif  // KINEMATICS_H
