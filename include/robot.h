#ifndef ROBOT_H
#define ROBOT_H

#include "absl/types/span.h"
#include "absl/strings/string_view.h"
#include <array>

namespace robotics {
class Robot {
 public:
  explicit Robot(
    absl::string_view name,
    absl::Span<const double> links_lengths,
    absl::Span<const double> base_position
  );

 private:
  std::string name_;
  std::array<double, 3> base_position_;
  std::vector<double> links_lengths_;

};
}  // namespace robotics

#endif  // ROBOT_H
