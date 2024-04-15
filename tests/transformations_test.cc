#include "transformations.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <array>
#include "absl/types/span.h"

namespace alaurens {

using ::testing::Pointwise;
using ::testing::DoubleEq;
using ::testing::DoubleNear;

namespace {

std::array<double, 16> kSixteenZeros{{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
}};

std::array<double, 16> kIdentityTransform{{
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
}};

}  // namespace

TEST(
    TransformationsTest,
    AssertApplyTransformationToVectorThrowsWithWrongArguements
) {
  std::array<double, 4> result;
  // Wrong vector size.
  ASSERT_THROW(
      ApplyTransformationToVector( 
          kSixteenZeros, {1.0, 0.0, 0.0}, absl::MakeSpan(result)
      ),
      std::invalid_argument
  );

  // Wrong transform size.
  ASSERT_THROW(
      ApplyTransformationToVector( 
          {0.0, 0.0}, {1.0, 0.0, 0.0, 1.0}, absl::MakeSpan(result)
      ),
      std::invalid_argument
  );

  // Wrong result size.
  std::array<double, 3> wrong_result;
  ASSERT_THROW(
      ApplyTransformationToVector( 
          kSixteenZeros, {1.0, 0.0, 0.0, 1.0}, absl::MakeSpan(wrong_result)
      ),
      std::invalid_argument
  );

  // Vec and result point to the same span.
  ASSERT_THROW(
      ApplyTransformationToVector(
          kSixteenZeros, result, absl::MakeSpan(result)
      ),
      std::invalid_argument
  );
}

TEST(TransformationsTest, RotateVectorWithSimpleRotation) {
  std::array<double, 4> vector({2.0, 4.5, -3.0, 1.0});
  std::array<double, 16> rotation_matrix_90_deg_z({
      0.0, -1.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0
  });
  std::array<double, 4> result;
  ApplyTransformationToVector(
      rotation_matrix_90_deg_z, vector, absl::MakeSpan(result)
  );

  EXPECT_THAT(result, Pointwise(DoubleEq(), {-4.5, 2.0, -3.0, 1.0}));
}

TEST(TransformationsTest, TranslateVector) {
  std::array<double, 4> vector({-2.3, 4.9, 0.0, 1.0});
  std::array<double, 16> pure_translation({
      1.0, 0.0, 0.0, 0.1,
      0.0, 1.0, 0.0, 36.0,
      0.0, 0.0, 1.0, -23.0,
      0.0, 0.0, 0.0, 1.0
  });
  std::array<double, 4> result;
  ApplyTransformationToVector(pure_translation, vector, absl::MakeSpan(result));

  EXPECT_THAT(result, Pointwise(DoubleEq(), {-2.2, 40.9, -23.0, 1.0}));
}

TEST(TransformationsTest, RotateAndTranslateVectorWithComplexMotion) {
  std::array<double, 4> vector({1.0, 1.0, 1.0, 1.0});

  // Rotate around an axis orthogonal to the vector. This will flip the vector.
  std::array<double, 16> transform;
  AxisAngleToTransform({-1.0, -1.0, 2.0}, M_PI, absl::MakeSpan(transform));

  // Translate the vector in y and z.
  transform[7] = 5.0;
  transform[11] = -3.0;


  std::array<double, 4> result;
  ApplyTransformationToVector(
      transform, vector, absl::MakeSpan(result)
  );

  EXPECT_THAT(result, Pointwise(DoubleEq(), {-1.0, 4.0, -4.0, 1.0}));
}

TEST(TransformationTest, AxisAngleToRotationMatrixRotationAroundZ){
  std::array<double, 3> axis({0.0, 0.0, 1.0});
  std::array<double, 16> result;

  AxisAngleToTransform(axis, M_PI / 3, absl::MakeSpan(result));
  EXPECT_THAT(
      result,
      Pointwise(
          DoubleNear(pow(10, -5)),
          {
              cos(M_PI / 3), -sin(M_PI / 3), 0.0, 0.0,
              sin(M_PI / 3), cos(M_PI / 3), 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 1.0
          }
      )
  );
}

TEST(TransformationTest, AxisAngleToRotationMatrixRotationComplexRotation){
  // Example 3.12 from the 2019 edition of "Modern Robotics Mechanics, Planning
  // and Control." By Kevin M.Lynch and Frank C.Park.
  std::array<double, 3> axis({0.0, 0.866, 0.5});
  std::array<double, 16> result;

  AxisAngleToTransform(axis, 0.524, absl::MakeSpan(result));
  EXPECT_THAT(
      result,
      Pointwise(
          DoubleNear(0.01),
          {
              0.866, -0.250, 0.433, 0.0,
              0.250, 0.967, 0.058, 0.0,
              -0.433, 0.058, 0.899, 0.0,
              0.0, 0.0, 0.0, 1.0
          }
      )
  );
}

TEST(TransformationTest, AxisAngleToRotationMatrixRotationZeroAngle){
  std::array<double, 3> axis({-2.3, 4.9, 0.0});
  std::array<double, 16> result;

  AxisAngleToTransform(axis, 0.0, absl::MakeSpan(result));
  EXPECT_THAT(result, Pointwise(DoubleNear(pow(10, -5)), kIdentityTransform));
}

TEST(TransformationTest, DHParametersToTransformThrows) {
  std::array<double, 4> result;
  // Wrong vector size.
  ASSERT_THROW(
      DHParametersToTransform(0.0, 0.0, 0.0, 0.0, absl::MakeSpan(result)),
      std::invalid_argument
  );
}

}  // namespace alaurens