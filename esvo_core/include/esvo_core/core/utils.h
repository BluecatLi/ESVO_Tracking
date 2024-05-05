#define UTILS_H_YFZCGEVA

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <functional>
#include <limits>
#include <cxxabi.h>



////////////////////////////////////////////////////////////////////////////////

#define UNUSED(x) ((void)(x))

////////////////////////////////////////////////////////////////////////////////

void image_gradient    (const cv::Mat& src, cv::Mat& dst);
