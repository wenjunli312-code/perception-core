#pragma once

#include <sensor_abstraction/camera.h>
#include <vector>
#include <string>

namespace perception_core {

struct DetectionBox {
    float x;      // top-left x in pixels
    float y;      // top-left y in pixels
    float width;
    float height;
    float score;
    int   class_id;
    std::string label;
};

class YoloDetector {
public:
    struct Config {
        float score_threshold = 0.5f;
        float nms_threshold   = 0.45f;
        int   input_width     = 640;
        int   input_height    = 640;
    };

    explicit YoloDetector(const Config& config = {});
    ~YoloDetector();

    // Run detection on a camera frame and return bounding boxes.
    std::vector<DetectionBox> detect(const sensor_abstraction::CameraFrame& frame) const;

private:
    Config config_;
};

} // namespace perception_core
