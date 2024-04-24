#include <yaml-cpp/yaml.h>

class CameraOmni
{
public:
    CameraOmni();
    // CameraOmni(const Eigen::Vector2i& camera_size,
    //         const Eigen::Vector2d& principal_point,
    //         const Eigen::Vector2d& focal_length,
    //         const Eigen::Vector2d clipping = Eigen::Vector2d(0,0));
    CameraOmni(YAML::Node config);

// private:

    // focal length
    double px, py;

    // principal point
    double u0, v0;

    //xi in omni
    double xi;

    // camera size
    double height, width;

    // initial pose
    double pose[7];

    // clipping paras
    float clip_near, clip_far;

    // distortion paras. Note: opencv calibration outputs [k1, k2, r1, r2, k3] while gcogre accepts [k1, k2, k3, r1, r2]
    double k[5];
};
