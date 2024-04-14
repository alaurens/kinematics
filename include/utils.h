#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

namespace alaurens {

template<typename T>
void ThrowIfSpanWrongSize(absl::Span<const T> data, int expected_size) {
  if(data.size() != expected_size){
    // Raising is maybe not the smartest thing to do in a RT system such as
    // as a robot, we might instead want to propagate the error to terminate
    // nicely.
    throw std::invalid_argument(
      absl::StrCat(
      "Data had a size of ", data.size(),
      ". Expected size ", expected_size, "."
    ));
  }
  return;
}
}  // namespace alaurens

#endif  // TRANSFORMATIONS_H
