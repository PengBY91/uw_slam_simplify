#include <gtest/gtest.h>

#include "measurement_api/frontend.hpp"

namespace {

class FakeMetricOpticalFrontend final : public uw::measurement_api::OpticalDepthFrontend {
 public:
  std::optional<uw::domain::MeasurementEvidence> Process(
      const uw::measurement_api::CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot&) override {
    uw::domain::OpticalDepthPriorMeasurement prior;
    *prior.mutable_reference_camera_frame() = bundle.primary.header().sensor_frame();
    prior.set_width(1);
    prior.set_height(1);
    prior.add_depth_m(bundle.secondary.has_value() ? 2.0f : 3.0f);
    prior.add_variance_m2(0.01f);
    prior.set_valid_mask(std::string{"\x01", 1});
    prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
    prior.set_producer_type(bundle.secondary.has_value() ? "fake_stereo" : "fake_monocular_metric");
    uw::domain::EvidenceId id;
    id.set_value("fake_optical_depth");
    return uw::domain::MakeEvidence(id, {}, prior, 1.0, "fake_metric_v1");
  }

  uw::domain::HealthReport Health() const override { return {}; }
};

}  // namespace

TEST(MeasurementApiContract, OpticalFrontendDoesNotRequireStereoAtInterfaceBoundary) {
  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary.mutable_header()->mutable_sensor_frame()->set_value("camera_left_link");
  FakeMetricOpticalFrontend frontend;
  const auto evidence = frontend.Process(bundle, {});
  ASSERT_TRUE(evidence.has_value());
  const auto& prior =
      uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(*evidence);
  EXPECT_EQ(prior.producer_type(), "fake_monocular_metric");
  EXPECT_EQ(prior.scale_status(), uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
}
