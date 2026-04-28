#include <perception_core/point_pillars.h>
#include <common_utils/logger.h>

namespace perception_core {

PointPillarsDetector::PointPillarsDetector(const Config& config)
    : config_(config)
{
    LOG_INFO("PointPillarsDetector initialised (score_thresh={})",
             config_.score_threshold);
}

PointPillarsDetector::~PointPillarsDetector() = default;

std::vector<Object3D> PointPillarsDetector::detect(
    const sensor_abstraction::LidarFrame& frame) const
{
    // Skeleton: a real implementation would:
    //   1. Voxelise the point cloud into pillars.
    //   2. Run the pillar feature network.
    //   3. Run the backbone + detection head.
    //   4. Decode anchors and apply NMS.
    LOG_DEBUG("PointPillarsDetector::detect — {} points",
              frame.point_count());

    std::vector<Object3D> results;
    // TODO: voxelisation → inference → decode → NMS
    return results;
}

} // namespace perception_core
