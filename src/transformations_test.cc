#include "transformations.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <array>
#include "absl/types/span.h"
#include "absl/strings/str_join.h"



namespace alaurens {

// Demonstrate some basic assertions.
TEST(TransformationsTest, RotateUnitVector) {
  std::array<double, 4> unit_vector({1.0, 0.0, 0.0, 1.0});
  std::array<double, 16> rotation_matrix_90_deg_z({
      0.0, -1.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
  });
  std::array<double, 4> result;
  RotateVector(
    absl::MakeConstSpan(rotation_matrix_90_deg_z),
    absl::MakeConstSpan(unit_vector),
    absl::MakeSpan(result)
  );

  EXPECT_THAT(
    result,
    ::testing::Pointwise(
        ::testing::DoubleEq(), std::array<double, 4>{0.0, 1.0, 0.0, 1.0}
    ))
  ;
}
}  // namespace alaurens