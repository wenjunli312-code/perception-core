#include <perception_core/lane_detector.h>
#include <common_utils/logger.h>

#define LOG_INFO(...)  // stub
#define LOG_DEBUG(...) // stub
#include <Eigen/QR>

namespace perception_core {

LaneDetector::LaneDetector() : config_() {
    LOG_INFO("LaneDetector initialised (roi_top_y={}, conf_thresh={:.2f})",
             config_.roi_top_y, config_.confidence_thresh);
}

LaneDetector::LaneDetector(const Config& config) : config_(config) {
    LOG_INFO("LaneDetector initialised (roi_top_y={}, conf_thresh={:.2f})",
             config_.roi_top_y, config_.confidence_thresh);
}

LaneDetector::~LaneDetector() = default;

std::vector<float> LaneDetector::fitPolynomial(
    const Eigen::MatrixX2f& points) const
{
    const int n  = static_cast<int>(points.rows());
    const int deg = config_.poly_degree;

    // Build Vandermonde matrix A where A(i,j) = y[i]^j.
    Eigen::MatrixXf A(n, deg + 1);
    for (int i = 0; i < n; ++i) {
        float yi = points(i, 1);
        float pw = 1.0f;
        for (int j = 0; j <= deg; ++j) {
            A(i, j) = pw;
            pw *= yi;
        }
    }

    Eigen::VectorXf x_vec = points.col(0);
    // Least-squares solve via Householder QR.
    Eigen::VectorXf coeffs = A.householderQr().solve(x_vec);

    std::vector<float> result(coeffs.data(), coeffs.data() + coeffs.size());
    return result;
}

std::vector<LaneLine> LaneDetector::detect(
    const sensor_abstraction::CameraFrame& frame) const
{
    // Skeleton: a real implementation would:
    //   1. Apply a perspective warp to get a bird's-eye-view image.
    //   2. Apply a colour/gradient threshold to isolate lane pixels.
    //   3. Use a sliding-window search to find lane pixel clusters.
    //   4. Call fitPolynomial() on each cluster.
    LOG_DEBUG("LaneDetector::detect — frame {}x{}",
              frame.width(), frame.height());

    std::vector<LaneLine> results;
    // TODO: BEV warp → thresholding → sliding window → fitPolynomial
    return results;
}

} // namespace perception_core
