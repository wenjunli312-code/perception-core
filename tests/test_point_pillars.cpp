#include <gtest/gtest.h>
#include <perception_core/point_pillars.h>
#include <sensor_abstraction/lidar_frame.h>

namespace {

TEST(PointPillarsDetectorTest, DefaultConfigConstruction) {
    perception_core::PointPillarsDetector detector;
    SUCCEED();
}

TEST(PointPillarsDetectorTest, CustomConfigConstruction) {
    perception_core::PointPillarsDetector::Config cfg;
    cfg.score_threshold = 0.6f;
    cfg.voxel_size_x    = 0.2f;
    cfg.voxel_size_y    = 0.2f;

    perception_core::PointPillarsDetector detector(cfg);
    SUCCEED();
}

TEST(PointPillarsDetectorTest, DetectReturnsEmptyOnEmptyCloud) {
    perception_core::PointPillarsDetector detector;
    sensor_abstraction::LidarFrame frame; // empty / default-constructed

    auto objects = detector.detect(frame);
    EXPECT_TRUE(objects.empty());
}

TEST(Object3DTest, FieldDefaults) {
    perception_core::Object3D obj{};
    EXPECT_FLOAT_EQ(obj.x,      0.0f);
    EXPECT_FLOAT_EQ(obj.y,      0.0f);
    EXPECT_FLOAT_EQ(obj.z,      0.0f);
    EXPECT_FLOAT_EQ(obj.score,  0.0f);
    EXPECT_EQ(obj.class_id, 0);
}

} // namespace
