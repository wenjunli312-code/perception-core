#pragma once

#include <perception_core/detector.h>
#include <perception_core/point_pillars.h>
#include <Eigen/Core>
#include <vector>

namespace perception_core {

// A fused object that combines camera and LiDAR detections.
struct FusedObject {
    Eigen::Vector3f position;   // 3D position (x, y, z) in meters
    Eigen::Vector3f dimensions; // (length, width, height) in meters
    float           yaw;        // heading angle in radians
    float           score;
    int             class_id;
};

class ObjectFusion {
public:
    struct Config {
        // Maximum distance (metres) between a 2D box projection and a 3D object
        // centre for them to be considered the same object.
        float association_threshold = 2.0f;
        // Kalman-filter-style weights for position blending
        float camera_weight = 0.3f;
        float lidar_weight  = 0.7f;

        Config() = default;
    };

    ObjectFusion();
    explicit ObjectFusion(const Config& config);

    // Fuse camera detections and LiDAR objects into a unified list.
    std::vector<FusedObject> fuse(
        const std::vector<DetectionBox>& camera_detections,
        const std::vector<Object3D>&     lidar_objects) const;

private:
    Config config_;

    // Compute a simple IoU-based association matrix.
    Eigen::MatrixXf computeAssociationMatrix(
        const std::vector<DetectionBox>& boxes,
        const std::vector<Object3D>&     objects) const;
};

} // namespace perception_core
