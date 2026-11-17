#include <winsock2.h>
#include <conio.h>

#include "StereoOdometry.hpp"
#include "CameraIdentity.hpp"
#include <myxreal/calibration_loader.h>
#include <myxreal/stereo_rectifier.h>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string_view>
#include <thread>

namespace
{
struct CameraSession
{
    CameraSession()
    {
        xreal::stereo::requireUltraCamera();
        if (stereo_init()) throw std::runtime_error("Stereo camera initialization failed");
        if (stereo_start_streaming())
        {
            stereo_shutdown();
            throw std::runtime_error("Air 2 Ultra stereo stream unavailable");
        }
    }
    ~CameraSession() { stereo_shutdown(); }
};

bool validPair(const StereoPair& raw, uint64_t lastTime)
{
    return raw.timestamp_ns > lastTime && raw.left.data && raw.right.data
        && raw.left.width == STEREO_EYE_WIDTH && raw.right.width == STEREO_EYE_WIDTH
        && raw.left.height == STEREO_EYE_HEIGHT && raw.right.height == STEREO_EYE_HEIGHT
        && std::isfinite(raw.sync_delta_ms) && raw.sync_delta_ms >= 0 && raw.sync_delta_ms <= 5;
}

int probe()
{
    CameraSession camera;
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    uint64_t lastTime{};
    unsigned pairs{}, accepted{};
    while (std::chrono::steady_clock::now() < end)
    {
        StereoPair pair{};
        if (stereo_poll_pair(&pair))
        {
            ++pairs;
            if (validPair(pair, lastTime)) { ++accepted; lastTime = pair.timestamp_ns; }
        }
        else std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "Stereo pairs in 5s: " << pairs << ", accepted: " << accepted << '\n';
    return accepted ? 0 : 1;
}

struct WinsockSession
{
    WinsockSession()
    {
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2,2), &data)) throw std::runtime_error("WSAStartup failed");
    }
    ~WinsockSession() { WSACleanup(); }
};

class PoseSender
{
public:
    PoseSender()
    {
        if (socket_ == INVALID_SOCKET) throw std::runtime_error("Failed to open pose socket");
        destination_.sin_family = AF_INET;
        destination_.sin_port = htons(45871);
        destination_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    }
    ~PoseSender() { closesocket(socket_); }

    void send(const std::optional<xreal::stereo::Pose>& pose, uint64_t timestamp)
    {
        // Wire basis (shared with the Unity publisher): X right, Y up, Z forward.
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        if (pose)
        {
            Eigen::Matrix3d r;
            for (int i = 0; i < 3; ++i)
            {
                p[i] = pose->position[i];
                for (int j = 0; j < 3; ++j) r(i,j) = pose->rotation(i,j);
            }
            p.y() = -p.y();
            const Eigen::DiagonalMatrix<double,3> basis(1., -1., 1.);
            q = Eigen::Quaterniond(basis*r*basis).normalized();
        }
        char packet[256];
        const int length = std::snprintf(packet, sizeof(packet),
            "1,%u,%d,%.9f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
            sequence_++, pose.has_value(), timestamp*1e-9, p.x(), p.y(), p.z(),
            q.x(), q.y(), q.z(), q.w());
        if (length <= 0 || length >= sizeof(packet)
            || sendto(socket_, packet, length, 0, reinterpret_cast<const sockaddr*>(&destination_),
                    sizeof(destination_)) != length)
            throw std::runtime_error("Failed to send local pose");
    }
private:
    SOCKET socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in destination_{};
    unsigned sequence_{};
};

int track(const char* path)
{
    std::unique_ptr<CalibrationData, decltype(&calibration_free)> calibration(
        calibration_load_json(path), calibration_free);
    if (!calibration || !calibration->is_valid || calibration->validation.failed)
        throw std::runtime_error("Invalid Air 2 Ultra factory camera calibration");
    StereoRectifier rectifier;
    rectifier.init(*calibration);
    if (!rectifier.initialized()) throw std::runtime_error("Stereo rectification failed");
    const cv::Matx33d k(rectifier.fx_rect(),0,rectifier.cx_rect(),
        0,rectifier.fy_rect(),rectifier.cy_rect(), 0,0,1);
    xreal::stereo::StereoOdometry tracker(k, rectifier.baseline());
    WinsockSession winsock;
    PoseSender sender;
    CameraSession camera;
    std::cout << "Waiting for stereo features. R: new origin, Esc: stop.\n";
    uint64_t lastTime{};
    bool tracking{};
    while (true)
    {
        if (_kbhit())
        {
            const int key = _getch();
            if (key == 27) break;
            if (key == 'r' || key == 'R') tracker.reset();
        }
        StereoPair raw{};
        if (!stereo_poll_pair(&raw))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        // Catch up to the newest pair without building a second frame queue.
        const int queued = stereo_available_pairs();
        for (int i = 0; i < queued; ++i) if (!stereo_poll_pair(&raw)) break;
        std::optional<xreal::stereo::Pose> pose;
        if (validPair(raw, lastTime))
        {
            lastTime = raw.timestamp_ns;
            const auto pair = rectifier.rectify(raw);
            const auto frame = [](const CameraFrame& f) {
                return cv::Mat(f.height, f.width, CV_8UC1, const_cast<uint8_t*>(f.data));
            };
            pose = tracker.update(frame(pair.left), frame(pair.right));
        }
        sender.send(pose, raw.timestamp_ns);
        if (tracking != pose.has_value())
        {
            tracking = pose.has_value();
            std::cout << (tracking ? "Tracking\n" : "Tracking lost\n");
        }
    }
    sender.send({}, lastTime);
    return 0;
}
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: xreal-native-stereo --probe | <factory-camera-calibration.json>\n";
        return 2;
    }
    try { return std::string_view(argv[1]) == "--probe" ? probe() : track(argv[1]); }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
