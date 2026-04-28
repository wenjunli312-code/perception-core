#include <gtest/gtest.h>
#include <perception_core/lane_detector.h>
#include <sensor_abstraction/camera_frame.h>

namespace {

TEST(LaneDetectorTest, DefaultConfigConstruction) {
    perception_core::LaneDetector detector;
    SUCCEED();
}

TEST(LaneDetectorTest, CustomConfigConstruction) {
    perception_core::LaneDetector::Config cfg;
    cfg.roi_top_y         = 300;
    cfg.confidence_thresh = 0.75f;
    cfg.poly_degree       = 2;

    perception_core::LaneDetector detector(cfg);
    SUCCEED();
}

TEST(LaneDetectorTest, DetectReturnsEmptyOnBlankFrame) {
    perception_core::LaneDetector detector;
    sensor_abstraction::CameraFrame frame;

    auto lanes = detector.detect(frame);
    EXPECT_TRUE(lanes.empty());
}

TEST(LaneLineTest, DefaultType) {
    perception_core::LaneLine lane;
    EXPECT_EQ(lane.type, perception_core::LaneLine::Type::UNKNOWN);
    EXPECT_FLOAT_EQ(lane.confidence, 0.0f);
}

} // namespace
