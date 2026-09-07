#include <gtest/gtest.h>

#include "runtime/run_manifest.hpp"

using namespace uw::runtime;


TEST(RunManifest, SerializesCalibrationAndDerivedCalibrationHashesAsIndependentFields) {
  RunManifest manifest;
  manifest.calibration_hash = "raw_hash_123";
  manifest.derived_calibration_hash = "derived_hash_456";
  const std::string json = manifest.ToJson();
  EXPECT_NE(json.find("\"calibration_hash\": \"raw_hash_123\""), std::string::npos);
  EXPECT_NE(json.find("\"derived_calibration_hash\": \"derived_hash_456\""), std::string::npos);
}

TEST(RunManifest, DerivedCalibrationHashDefaultsToEmpty) {
  RunManifest manifest;
  EXPECT_TRUE(manifest.derived_calibration_hash.empty());
  EXPECT_NE(manifest.ToJson().find("\"derived_calibration_hash\": \"\""), std::string::npos);
}
