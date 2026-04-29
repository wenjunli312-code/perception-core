#pragma once

#include <sensor_abstraction/camera.h>
#include <Eigen/Core>
#include <vector>

namespace perception_core {

struct LaneLine {
    // Polynomial coefficients for the lane curve in the image plane:
    //   x = coeffs[0] + coeffs[1]*y + coeffs[2]*y^2 + coeffs[3]*y^3
    std::vector<float> coeffs;
    float confidence;

    enum class Type { SOLID, DASHED, UNKNOWN };
    Type type = Type::UNKNOWN;
};

class LaneDetector {
public:
    struct Config {
        int   roi_top_y         = 360;  // pixels from top where ROI starts
        float confidence_thresh = 0.6f;
        int   poly_degree       = 3;    // polynomial degree for lane fitting

        Config() = default;
    };

    LaneDetector();
    explicit LaneDetector(const Config& config);
    ~LaneDetector();

    // Detect lane lines in a camera frame.
    // Returns left and right lane lines (if found).
    std::vector<LaneLine> detect(const sensor_abstraction::CameraFrame& frame) const;

private:
    Config config_;

    // Fit a polynomial to a set of (x, y) points.
    std::vector<float> fitPolynomial(const Eigen::MatrixX2f& points) const;
};

} // namespace perception_core
