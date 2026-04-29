#include <perception_core/detector.h>
#include <common_utils/logger.h>

#define LOG_INFO(...)   // stub: no-op when no logger
#define LOG_DEBUG(...)  // stub: no-op when no logger

namespace perception_core {

YoloDetector::YoloDetector() : config_() {
    LOG_INFO("YoloDetector initialised (score_thresh={}, nms_thresh={})",
             config_.score_threshold, config_.nms_threshold);
}

YoloDetector::YoloDetector(const Config& config) : config_(config) {
    LOG_INFO("YoloDetector initialised (score_thresh={}, nms_thresh={})",
             config_.score_threshold, config_.nms_threshold);
}

YoloDetector::~YoloDetector() = default;

std::vector<DetectionBox> YoloDetector::detect(
    const sensor_abstraction::CameraFrame& frame) const
{
    // Skeleton: a real implementation would run an inference engine (e.g. TensorRT)
    // on frame.data() and decode the raw network output into boxes.
    LOG_DEBUG("YoloDetector::detect — frame {}x{}",
              frame.width(), frame.height());

    std::vector<DetectionBox> results;
    // TODO: inference + NMS
    return results;
}

} // namespace perception_core
