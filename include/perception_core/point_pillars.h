#pragma once

#include <sensor_abstraction/lidar_frame.h>
#include <vector>

namespace perception_core {

struct Object3D {
    float x;      // center x in meters
    float y;      // center y in meters
    float z;      // center z in meters
    float length;
    float width;
    float height;
    float yaw;    // heading angle in radians
    float score;
    int   class_id;
};

class PointPillarsDetector {
public:
    struct Config {
        float score_threshold  = 0.4f;
        float voxel_size_x     = 0.16f;
        float voxel_size_y     = 0.16f;
        float voxel_size_z     = 4.0f;
        float point_cloud_range_x_min = -39.68f;
        float point_cloud_range_x_max =  39.68f;
        float point_cloud_range_y_min = -39.68f;
        float point_cloud_range_y_max =  39.68f;

        Config() = default;
    };

    PointPillarsDetector();
    explicit PointPillarsDetector(const Config& config);
    ~PointPillarsDetector();

    // Run detection on a LiDAR point cloud frame and return 3D objects.
    std::vector<Object3D> detect(const sensor_abstraction::LidarFrame& frame) const;

private:
    Config config_;
};

} // namespace perception_core
