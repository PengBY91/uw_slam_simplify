#include "sensor_models/camera_rectifier.hpp"

#include <optional>


namespace uw::sensor_models {

bool PlumbBobDistortion::IsIdentity() const {
  return k1 == 0.0 && k2 == 0.0 && p1 == 0.0 && p2 == 0.0 && k3 == 0.0;
}

std::optional<PlumbBobDistortion> PlumbBobDistortion::FromIntrinsics(
    const uw::domain::CameraIntrinsics& intrinsics) {
  if (!intrinsics.distortion_model().empty() && intrinsics.distortion_model() != "plumb_bob") {
    return std::nullopt;
  }
  const int n = intrinsics.distortion_size();
  if (n != 0 && n != 4 && n != 5) return std::nullopt;

  PlumbBobDistortion distortion;
  if (n >= 1) distortion.k1 = intrinsics.distortion(0);
  if (n >= 2) distortion.k2 = intrinsics.distortion(1);
  if (n >= 3) distortion.p1 = intrinsics.distortion(2);
  if (n >= 4) distortion.p2 = intrinsics.distortion(3);
  if (n >= 5) distortion.k3 = intrinsics.distortion(4);
  return distortion;
}

Eigen::Vector2d ApplyPlumbBobDistortion(const PlumbBobDistortion& distortion,
                                         const Eigen::Vector2d& normalized_undistorted) {
  const double x = normalized_undistorted.x();
  const double y = normalized_undistorted.y();
  const double r2 = x * x + y * y;
  const double radial = 1.0 + distortion.k1 * r2 + distortion.k2 * r2 * r2 + distortion.k3 * r2 * r2 * r2;
  const double x_distorted = x * radial + 2.0 * distortion.p1 * x * y + distortion.p2 * (r2 + 2.0 * x * x);
  const double y_distorted = y * radial + distortion.p1 * (r2 + 2.0 * y * y) + 2.0 * distortion.p2 * x * y;
  return Eigen::Vector2d(x_distorted, y_distorted);
}

}  // namespace uw::sensor_models
