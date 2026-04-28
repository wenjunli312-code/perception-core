#include <gtest/gtest.h>
#include <perception_core/detector.h>
#include <sensor_abstraction/camera_frame.h>

namespace {

TEST(YoloDetectorTest, DefaultConfigConstruction) {
    perception_core::YoloDetector detector;
    // Constructor must not throw.
    SUCCEED();
}

TEST(YoloDetectorTest, CustomConfigConstruction) {
    perception_core::YoloDetector::Config cfg;
    cfg.score_threshold = 0.7f;
    cfg.nms_threshold   = 0.3f;
    cfg.input_width     = 416;
    cfg.input_height    = 416;

    perception_core::YoloDetector detector(cfg);
    SUCCEED();
}

TEST(YoloDetectorTest, DetectReturnsEmptyOnBlankFrame) {
    perception_core::YoloDetector detector;
    sensor_abstraction::CameraFrame frame; // empty / default-constructed

    auto boxes = detector.detect(frame);
    // Skeleton implementation returns no detections.
    EXPECT_TRUE(boxes.empty());
}

TEST(DetectionBoxTest, FieldDefaults) {
    perception_core::DetectionBox box{};
    EXPECT_FLOAT_EQ(box.x,      0.0f);
    EXPECT_FLOAT_EQ(box.y,      0.0f);
    EXPECT_FLOAT_EQ(box.width,  0.0f);
    EXPECT_FLOAT_EQ(box.height, 0.0f);
    EXPECT_FLOAT_EQ(box.score,  0.0f);
    EXPECT_EQ(box.class_id, 0);
}

} // namespace
