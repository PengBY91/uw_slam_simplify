#pragma once

#include <optional>

#include <Eigen/Core>

#include "domain/domain.hpp"

namespace uw::sensor_models {

// Plumb-bob (radial-tangential) lens distortion, matching
// CameraIntrinsics.distortion's convention when distortion_model ==
// "plumb_bob" (this repo's only supported model, and the default written by
// uw::runtime::LoadRigConfig — see configs/rig/example_auv_real_camera.yaml
// for real calibrated coefficients): [k1, k2, p1, p2] (4 values) or
// [k1, k2, p1, p2, k3] (5 values). This is the same convention ROS/OpenCV
// call "plumb_bob" — deliberately not the OpenCV library itself, which this
// repo does not depend on (see cmake/Dependencies.cmake).
struct PlumbBobDistortion {
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  double k3 = 0.0;

  // True when every coefficient is exactly zero — i.e. the source is
  // already distortion-free (true of every synthetic rig in configs/rig/
  // today) and UndistortImage() below can skip processing entirely.
  bool IsIdentity() const;

  // Returns std::nullopt if intrinsics.distortion_model() is set to
  // something other than "plumb_bob", or distortion_size() is a length
  // this model doesn't recognize (must be 0, 4, or 5).
  static std::optional<PlumbBobDistortion> FromIntrinsics(
      const uw::domain::CameraIntrinsics& intrinsics);
};

// Applies the forward plumb-bob distortion model to a point already
// normalized by the camera's inverse K (i.e. (u - cx) / fx, (v - cy) / fy
// for an UNDISTORTED pixel (u, v)), returning the corresponding DISTORTED
// normalized coordinate. Exposed standalone (not just inside UndistortImage)
// so the polynomial itself can be unit-tested against known-good values
// independent of image warping/interpolation.
Eigen::Vector2d ApplyPlumbBobDistortion(const PlumbBobDistortion& distortion,
                                         const Eigen::Vector2d& normalized_undistorted);

}  // namespace uw::sensor_models
