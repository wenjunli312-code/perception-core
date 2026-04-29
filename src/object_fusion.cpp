#include <perception_core/object_fusion.h>
#include <common_utils/logger.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

#define LOG_INFO(...)   // stub
#define LOG_DEBUG(...)  // stub

namespace perception_core {

ObjectFusion::ObjectFusion() : config_() {
    LOG_INFO("ObjectFusion initialised (assoc_thresh={:.2f}, lidar_w={:.2f})",
             config_.association_threshold, config_.lidar_weight);
}

ObjectFusion::ObjectFusion(const Config& config) : config_(config) {
    LOG_INFO("ObjectFusion initialised (assoc_thresh={:.2f}, lidar_w={:.2f})",
             config_.association_threshold, config_.lidar_weight);
}

Eigen::MatrixXf ObjectFusion::computeAssociationMatrix(
    const std::vector<DetectionBox>& boxes,
    const std::vector<Object3D>&     objects) const
{
    const int rows = static_cast<int>(boxes.size());
    const int cols = static_cast<int>(objects.size());
    Eigen::MatrixXf cost(rows, cols);

    for (int i = 0; i < rows; ++i) {
        // Project the 2D box centre back to a rough ground-plane position.
        float cx = boxes[i].x + boxes[i].width  * 0.5f;
        float cy = boxes[i].y + boxes[i].height * 0.5f;

        for (int j = 0; j < cols; ++j) {
            // Simple Euclidean distance in the image coordinate proxy.
            float dx = cx - objects[j].x;
            float dy = cy - objects[j].y;
            cost(i, j) = std::sqrt(dx * dx + dy * dy);
        }
    }
    return cost;
}

std::vector<FusedObject> ObjectFusion::fuse(
    const std::vector<DetectionBox>& camera_detections,
    const std::vector<Object3D>&     lidar_objects) const
{
    LOG_DEBUG("ObjectFusion::fuse — {} camera dets, {} lidar objs",
              camera_detections.size(), lidar_objects.size());

    std::vector<FusedObject> fused;
    fused.reserve(lidar_objects.size());

    // Start from LiDAR objects (primary source for 3D geometry).
    std::vector<bool> matched(camera_detections.size(), false);

    if (!camera_detections.empty()) {
        Eigen::MatrixXf cost = computeAssociationMatrix(camera_detections, lidar_objects);

        for (int j = 0; j < static_cast<int>(lidar_objects.size()); ++j) {
            const auto& obj = lidar_objects[j];

            // Find the nearest unmatched camera detection.
            int   best_i  = -1;
            float best_d  = config_.association_threshold;
            for (int i = 0; i < static_cast<int>(camera_detections.size()); ++i) {
                if (!matched[i] && cost(i, j) < best_d) {
                    best_d = cost(i, j);
                    best_i = i;
                }
            }

            FusedObject fo;
            fo.position    = {obj.x, obj.y, obj.z};
            fo.dimensions  = {obj.length, obj.width, obj.height};
            fo.yaw         = obj.yaw;
            fo.class_id    = obj.class_id;

            if (best_i >= 0) {
                matched[best_i] = true;
                // Blend scores.
                fo.score = config_.lidar_weight  * obj.score
                         + config_.camera_weight * camera_detections[best_i].score;
            } else {
                fo.score = obj.score;
            }

            fused.push_back(fo);
        }
    } else {
        for (const auto& obj : lidar_objects) {
            FusedObject fo;
            fo.position   = {obj.x, obj.y, obj.z};
            fo.dimensions = {obj.length, obj.width, obj.height};
            fo.yaw        = obj.yaw;
            fo.score      = obj.score;
            fo.class_id   = obj.class_id;
            fused.push_back(fo);
        }
    }

    LOG_DEBUG("ObjectFusion::fuse — {} fused objects", fused.size());
    return fused;
}

} // namespace perception_core
