#include "robot.h"
#include "absl/strings/str_join.h"
#include <iostream>
#include "absl/types/span.h"
#include "absl/algorithm/container.h"

#include <Eigen/Dense>

namespace robotics {

Robot::Robot(
  absl::string_view name,
  absl::Span<const double> links_lengths,
  absl::Span<const double> base_position
):name_(name) {
  links_lengths_.resize(links_lengths.size());
  if (base_position.size() != 3) {
    throw std::invalid_argument("Robot::Robot: The base_position must have size 3.");
  }
  absl::c_copy(base_position_, base_position_.begin());
  std::cout << absl::StrJoin(base_position_, " ") <<std::endl;

  absl::c_copy(links_lengths, links_lengths_.begin());
}

}  // namespace robotics

