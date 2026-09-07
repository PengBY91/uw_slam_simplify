#include "sensor_models/camera_rectifier.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using uw::sensor_models::ApplyPlumbBobDistortion;
using uw::sensor_models::PlumbBobDistortion;

namespace {

uw::domain::CameraIntrinsics MakeIntrinsics(uint32_t width, uint32_t height, double fx, double fy, double cx,
                                             double cy, std::vector<double> distortion,
                                             const std::string& distortion_model = "plumb_bob") {
  uw::domain::CameraIntrinsics intrinsics;
  intrinsics.set_width(width);
  intrinsics.set_height(height);
  for (double v : {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}) intrinsics.add_k_matrix_row_major(v);
  for (double v : distortion) intrinsics.add_distortion(v);
  intrinsics.set_distortion_model(distortion_model);
  return intrinsics;
}

}  // namespace

TEST(PlumbBobDistortionTest, FromIntrinsicsAcceptsZeroFourAndFiveCoefficients) {
  const auto zero = PlumbBobDistortion::FromIntrinsics(MakeIntrinsics(64, 64, 100, 100, 32, 32, {}));
  ASSERT_TRUE(zero.has_value());
  EXPECT_TRUE(zero->IsIdentity());

  const auto four =
      PlumbBobDistortion::FromIntrinsics(MakeIntrinsics(64, 64, 100, 100, 32, 32, {0.1, -0.05, 0.001, -0.002}));
  ASSERT_TRUE(four.has_value());
  EXPECT_DOUBLE_EQ(four->k1, 0.1);
  EXPECT_DOUBLE_EQ(four->k2, -0.05);
  EXPECT_DOUBLE_EQ(four->p1, 0.001);
  EXPECT_DOUBLE_EQ(four->p2, -0.002);
  EXPECT_DOUBLE_EQ(four->k3, 0.0);
  EXPECT_FALSE(four->IsIdentity());

  const auto five = PlumbBobDistortion::FromIntrinsics(
      MakeIntrinsics(64, 64, 100, 100, 32, 32, {0.017961, -0.042131, -0.000550, 0.001529, 0.023008}));
  ASSERT_TRUE(five.has_value());
  EXPECT_DOUBLE_EQ(five->k3, 0.023008);
}

TEST(PlumbBobDistortionTest, FromIntrinsicsRejectsUnsupportedLengthOrModel) {
  EXPECT_EQ(PlumbBobDistortion::FromIntrinsics(MakeIntrinsics(64, 64, 100, 100, 32, 32, {0.1, 0.2})), std::nullopt);
  EXPECT_EQ(PlumbBobDistortion::FromIntrinsics(
                MakeIntrinsics(64, 64, 100, 100, 32, 32, {0.1, 0.2, 0.3, 0.4}, "fisheye")),
            std::nullopt);
  // Empty distortion_model string (proto3 default) is treated as plumb_bob.
  EXPECT_NE(PlumbBobDistortion::FromIntrinsics(MakeIntrinsics(64, 64, 100, 100, 32, 32, {}, "")), std::nullopt);
}

TEST(PlumbBobDistortionTest, ApplyIsIdentityForZeroCoefficients) {
  const PlumbBobDistortion identity;
  const Eigen::Vector2d p(0.3, -0.2);
  const Eigen::Vector2d out = ApplyPlumbBobDistortion(identity, p);
  EXPECT_NEAR(out.x(), p.x(), 1e-12);
  EXPECT_NEAR(out.y(), p.y(), 1e-12);
}

TEST(PlumbBobDistortionTest, ApplyMatchesHandComputedRadialValue) {
  // Pure radial (p1=p2=k3=0): x_d = x * (1 + k1*r2 + k2*r2^2).
  PlumbBobDistortion d;
  d.k1 = 0.1;
  d.k2 = 0.02;
  const Eigen::Vector2d p(0.4, 0.0);
  const double r2 = 0.4 * 0.4;
  const double expected_x = 0.4 * (1.0 + 0.1 * r2 + 0.02 * r2 * r2);
  const Eigen::Vector2d out = ApplyPlumbBobDistortion(d, p);
  EXPECT_NEAR(out.x(), expected_x, 1e-12);
  EXPECT_NEAR(out.y(), 0.0, 1e-12);
}
