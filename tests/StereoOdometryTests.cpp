#include "StereoOdometry.hpp"
#include <iostream>
#include <stdexcept>

namespace
{
void require(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(message);
}

// Project distinct textured targets at several depths into a moving stereo rig.
cv::Mat image(double x, double y = 0, double z = 0, cv::Vec3d angles = {})
{
    cv::RNG rng(124);
    cv::Mat out(480,640,CV_8UC1,cv::Scalar(25));
    cv::Matx33d worldFromCamera;
    cv::Rodrigues(angles, worldFromCamera);
    for (int v = 50; v < 440; v += 43)
        for (int u = 58; u < 602; u += 45)
        {
            const double depth = 1.5 + rng.uniform(0.,2.);
            const double dx = (u-320)*depth/400-x, dy = (v-240)*depth/400-y;
            const cv::Vec3d p = worldFromCamera.t() * cv::Vec3d(dx, dy, depth-z);
            const int px = cvRound(320+400*p[0]/p[2]);
            const int py = cvRound(240+400*p[1]/p[2]);
            cv::Mat tile(19,19,CV_8UC1);
            rng.fill(tile,cv::RNG::UNIFORM,40,250);
            if (px>=9 && py>=9 && px+9<out.cols && py+9<out.rows)
                tile.copyTo(out(cv::Rect(px-9,py-9,19,19)));
        }
    return out;
}
}

int main()
{
    try
    {
        xreal::stereo::StereoOdometry vo({400,0,320,0,400,240,0,0,1},0.10);
        require(!vo.update({},{}), "Empty pair reported tracking");
        require(!vo.update(image(0),image(0)), "Zero disparity reported tracking");
        auto origin = vo.update(image(0),image(0.1));
        require(origin && cv::norm(origin->position)<1e-9, "Missing metric origin");
        const cv::Mat blank = cv::Mat::zeros(480,640,CV_8U);
        require(!vo.update(blank,blank), "Blank images reported movement");
        auto pose = vo.update(image(.045,.023,.04), image(.145,.023,.04));
        require(pose && cv::norm(pose->position-cv::Vec3d(.045,.023,.04))<.025,
            "Recovered translation has wrong scale, sign or axes");
        const cv::Vec3d angles(.035,-.045,.04);
        cv::Matx33d orientation;
        cv::Rodrigues(angles, orientation);
        const cv::Vec3d l(.045,.023,.04), r = l + orientation * cv::Vec3d(.1,0,0);
        auto rotated = vo.update(image(l[0],l[1],l[2],angles), image(r[0],r[1],r[2],angles));
        require(rotated && cv::norm(rotated->rotation-orientation)<.035,
            "Camera pitch, yaw and roll not recovered");
        vo.reset();
        origin = vo.update(image(.045),image(.145));
        require(origin && cv::norm(origin->position)<1e-9, "Recenter failed");
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
    return 0;
}
