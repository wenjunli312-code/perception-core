#include <gtest/gtest.h>
#include <perception_core/object_fusion.h>

namespace {

TEST(ObjectFusionTest, DefaultConfigConstruction) {
    perception_core::ObjectFusion fusion;
    SUCCEED();
}

TEST(ObjectFusionTest, FuseEmptyInputs) {
    perception_core::ObjectFusion fusion;
    auto result = fusion.fuse({}, {});
    EXPECT_TRUE(result.empty());
}

TEST(ObjectFusionTest, FuseLidarOnlyPassesThrough) {
    perception_core::ObjectFusion fusion;

    perception_core::Object3D obj{};
    obj.x = 5.0f; obj.y = 2.0f; obj.z = 0.5f;
    obj.length = 4.5f; obj.width = 1.8f; obj.height = 1.5f;
    obj.score = 0.9f; obj.class_id = 1;

    auto result = fusion.fuse({}, {obj});

    ASSERT_EQ(result.size(), 1u);
    EXPECT_FLOAT_EQ(result[0].position.x(), obj.x);
    EXPECT_FLOAT_EQ(result[0].position.y(), obj.y);
    EXPECT_FLOAT_EQ(result[0].score, obj.score);
}

TEST(ObjectFusionTest, FuseMatchedPairBlendsScore) {
    perception_core::ObjectFusion::Config cfg;
    cfg.association_threshold = 1000.0f; // force association
    cfg.camera_weight = 0.3f;
    cfg.lidar_weight  = 0.7f;
    perception_core::ObjectFusion fusion(cfg);

    perception_core::DetectionBox box{};
    box.x = 100.0f; box.y = 200.0f;
    box.width = 80.0f; box.height = 60.0f;
    box.score = 0.8f; box.class_id = 1;

    perception_core::Object3D obj{};
    // Place object centre near box centre
    obj.x = 140.0f; obj.y = 230.0f; obj.z = 0.0f;
    obj.length = 4.5f; obj.width = 1.8f; obj.height = 1.5f;
    obj.score = 0.9f; obj.class_id = 1;

    auto result = fusion.fuse({box}, {obj});

    ASSERT_EQ(result.size(), 1u);
    float expected_score = 0.7f * 0.9f + 0.3f * 0.8f;
    EXPECT_NEAR(result[0].score, expected_score, 1e-5f);
}

} // namespace
