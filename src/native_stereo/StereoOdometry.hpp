#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <optional>

namespace xreal::stereo
{
// Camera convention: +X right, +Y down, +Z forward. Distances in metres.
struct Pose
{
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Vec3d position{};
};

class StereoOdometry
{
public:
    StereoOdometry(cv::Matx33d intrinsics, double baselineMetres);
    std::optional<Pose> update(const cv::Mat& left, const cv::Mat& right);
    void reset();

private:
    cv::Matx33d intrinsics_;
    double baseline_;
    cv::Ptr<cv::ORB> detector_ = cv::ORB::create(1400);
    // Metric points and descriptors in the last accepted reference camera.
    std::vector<cv::Point3f> referencePoints_;
    cv::Mat referenceDescriptors_;
    Pose referencePose_;
};
}
