#ifndef ESVO_CORE_CONTAINER_CAMERASYSTEM_H
#define ESVO_CORE_CONTAINER_CAMERASYSTEM_H

#include <string>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>

namespace esvo_core
{
namespace container
{
class OmniCamera
{
  public:
  OmniCamera();
  virtual ~OmniCamera();
  using Ptr = std::shared_ptr<OmniCamera>;

  OmniCamera(YAML::Node config);

  void cam2World(const Eigen::Vector2d &x, double invDepth, Eigen::Vector3d &p);

  void world2Cam(const Eigen::Vector3d &p, Eigen::Vector2d &x);

  public:
  
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

}
}

#endif //ESVO_CORE_CONTAINER_CAMERASYSTEM_H