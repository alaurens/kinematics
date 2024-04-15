#ifndef UTILS_H
#define UTILS_H

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

namespace alaurens {

// Will throw an `std::invalid_argument if the size of `data` does not match
// `expected_size`. Note that raising is maybe not the smartest thing to do in
// a RT system like the one usually used to control a robot, we might instead
// want to propagate the error to terminate properly .
template<typename T>
void ThrowIfSpanWrongSize(absl::Span<const T> data, int expected_size) {
  if(data.size() != expected_size)
    throw std::invalid_argument(
      absl::StrCat(
          "Data had a size of ", data.size(),
          ". Expected size ", expected_size, "."
      )
    );
  return;
}
}  // namespace alaurens

#endif  // TRANSFORMATIONS_H
