#include "StereoOdometry.hpp"

#include <cmath>
#include <stdexcept>

namespace xreal::stereo
{
namespace
{
constexpr int minimumPoints = 16;

std::vector<cv::DMatch> matches(const cv::Mat& query, const cv::Mat& train)
{
    std::vector<cv::DMatch> accepted;
    if (query.rows < 2 || train.rows < 2) return accepted;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> candidates;
    std::vector<cv::DMatch> reverse;
    matcher.knnMatch(query, train, candidates, 2);
    matcher.match(train, query, reverse);
    for (const auto& pair : candidates)
    {
        if (pair.size() != 2) continue;
        const auto& best = pair.front();
        if (best.distance < 0.75f * pair[1].distance
            && reverse[best.trainIdx].trainIdx == best.queryIdx)
            accepted.push_back(best);
    }
    return accepted;
}
}

StereoOdometry::StereoOdometry(cv::Matx33d intrinsics, double baselineMetres)
    : intrinsics_(intrinsics), baseline_(baselineMetres)
{
    if (!cv::checkRange(intrinsics_) || intrinsics_(0,0) <= 0 || intrinsics_(1,1) <= 0
        || !std::isfinite(baseline_) || baseline_ <= 0)
        throw std::invalid_argument("Invalid rectified stereo calibration.");
}

void StereoOdometry::reset()
{
    referencePoints_.clear();
    referenceDescriptors_.release();
    referencePose_ = {};
}

std::optional<Pose> StereoOdometry::update(const cv::Mat& left, const cv::Mat& right)
{
    if (left.empty() || left.size() != right.size()
        || left.type() != CV_8UC1 || right.type() != CV_8UC1) return {};
    std::vector<cv::KeyPoint> lk, rk;
    cv::Mat ld, rd;
    detector_->detectAndCompute(left, cv::noArray(), lk, ld);
    detector_->detectAndCompute(right, cv::noArray(), rk, rd);
    std::vector<cv::Point3f> points;
    cv::Mat descriptors;
    for (const auto& m : matches(ld, rd))
    {
        const auto a = lk[m.queryIdx].pt;
        const auto b = rk[m.trainIdx].pt;
        const double disparity = a.x - b.x;
        if (disparity < 2 || std::abs(a.y - b.y) > 1.5) continue;
        const double z = intrinsics_(0,0) * baseline_ / disparity;
        if (z < 0.15 || z > 12.0) continue;
        points.emplace_back(static_cast<float>((a.x - intrinsics_(0,2))*z/intrinsics_(0,0)),
            static_cast<float>((a.y - intrinsics_(1,2))*z/intrinsics_(1,1)), static_cast<float>(z));
        descriptors.push_back(ld.row(m.queryIdx));
    }
    if (points.size() < minimumPoints) return {};

    Pose pose = referencePose_;
    bool replaceReference = referencePoints_.empty();
    if (!replaceReference)
    {
        std::vector<cv::Point3f> objects;
        std::vector<cv::Point2f> pixels;
        for (const auto& m : matches(referenceDescriptors_, ld))
        {
            objects.push_back(referencePoints_[m.queryIdx]);
            pixels.push_back(lk[m.trainIdx].pt);
        }
        if (objects.size() < minimumPoints) return {};
        cv::Vec3d r, t;
        std::vector<int> inliers;
        if (!cv::solvePnPRansac(objects, pixels, intrinsics_, cv::noArray(), r, t,
                false, 120, 2.0f, 0.995, inliers, cv::SOLVEPNP_EPNP)
            || inliers.size() < minimumPoints || inliers.size()*2 < objects.size()) return {};
        cv::Matx33d cameraFromReference;
        cv::Rodrigues(r, cameraFromReference);
        pose.rotation = referencePose_.rotation * cameraFromReference.t();
        pose.position = referencePose_.position - pose.rotation * t;
        if (!cv::checkRange(pose.rotation) || !cv::checkRange(pose.position)) return {};
        // Keep the same map while stationary to avoid integrating image noise.
        replaceReference = cv::norm(t) > 0.08 || cv::norm(r) > 0.14
            || inliers.size()*3 < referencePoints_.size();
    }
    if (replaceReference)
    {
        referencePoints_ = std::move(points);
        referenceDescriptors_ = std::move(descriptors);
        referencePose_ = pose;
    }
    return pose;
}
}
