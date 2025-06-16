#ifndef ESVO_CORE_TRACKING_H
#define ESVO_CORE_TRACKING_H

#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <tf2_ros/transform_broadcaster.h>

#include <esvo_core/container/CameraSystem.h>
#include <esvo_core/core/RegProblemLM.h>
#include <esvo_core/core/RegProblemSolverLM.h>
#include <esvo_core/tools/utils.h>
#include <esvo_core/tools/Visualization.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <map>
#include <deque>
#include <mutex>
#include <future>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

//yufan added for generalizing omni camera model

#include <ORFCV/gcOgre.h>
#include <ORFCV/COmni.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPoseVector.h>
#include <esvo_core/core/camera_omni.h>
#include <esvo_core/core/keyframe_omni.h>
#include <esvo_core/core/utils.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <termios.h>   // 终端控制
#include <unistd.h>    // STDIN_FILENO 和 read/write
#include <fcntl.h>     // fcntl
#include <GL/gl.h>


namespace esvo_core
{
using namespace core;
enum TrackingStatus
{
  IDLE,
  WORKING
};

class esvo_Tracking
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  esvo_Tracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~esvo_Tracking();

  // functions regarding tracking
  void TrackingLoop();
  bool refDataTransferring();
  bool curDataTransferring();// These two data transferring functions are decoupled because the data are not updated at the same frequency.

  // topic callback functions
  void refMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void timeSurfaceCallback(
    const sensor_msgs::ImageConstPtr& time_surface_left,
    const sensor_msgs::ImageConstPtr& time_surface_right);
  void pointsetCallback(const sensor_msgs::ImageConstPtr& point_set);
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

  // results
  void publishPose(const ros::Time& t, Transformation& tr);
  void publishPath(const ros::Time& t, Transformation& tr);
  void saveTrajectory(const std::string &resultDir);

  // utils
  void reset();
  void clearEventQueue();
  void stampedPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  bool getPoseAt(
    const ros::Time &t,
    esvo_core::Transformation &Tr,// T_world_something
    const std::string& source_frame );

  private:
  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;

  // subscribers and publishers
  ros::Subscriber events_left_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber Point_set_sub_;
  message_filters::Subscriber<sensor_msgs::Image> TS_left_sub_, TS_right_sub_;
  // message_filters::Subscriber<sensor_msgs::Image> TS_left_sub_;
  ros::Subscriber stampedPose_sub_;
  image_transport::Publisher reprojMap_pub_left_;
  // yufan added
  image_transport::Publisher pointSet_pub_;

  // publishers
  ros::Publisher pose_pub_, path_pub_, evs_pub_;

  // results
  nav_msgs::Path path_;
  std::list<Eigen::Matrix<double,4,4>, Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > > lPose_;
  std::list<std::string> lTimestamp_;

  // Time Surface sync policy
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactSyncPolicy;
  message_filters::Synchronizer<ExactSyncPolicy> TS_sync_;

  // offline data
  std::string dvs_frame_id_;
  std::string world_frame_id_;
  std::string calibInfoDir_;
  CameraSystem::Ptr camSysPtr_;

  // inter-thread management
  std::mutex data_mutex_;

  // online data
  EventQueue events_left_;
  TimeSurfaceHistory TS_history_;
  size_t TS_id_;
  std::shared_ptr<tf::Transformer> tf_;
  RefPointCloudMap refPCMap_;
  RefFrame ref_;
  CurFrame cur_;

  /**** offline parameters ***/
  size_t tracking_rate_hz_;
  size_t TS_HISTORY_LENGTH_;
  size_t REF_HISTORY_LENGTH_;
  bool bSaveTrajectory_;
  bool bVisualizeTrajectory_;
  std::string resultPath_;
  std::string evsModelPath_;
  std::string camIntrinsicPath_;

  Eigen::Matrix<double, 4, 4> T_world_ref_;
  Eigen::Matrix<double, 4, 4> T_world_cur_;

  /*** system objects ***/
  RegProblemType rpType_;
  TrackingStatus ets_;
  std::string ESVO_System_Status_;
  RegProblemConfig::Ptr rpConfigPtr_;
  RegProblemSolverLM rpSolver_;

  //Yufan added for omni camera model
  gcOgre *moteur, *moteur2;
  CameraOmni cam_omni;
  COmni *cam;
  vpHomogeneousMatrix toHomogeneousMatrix(double *s);
  vpHomogeneousMatrix c0Mo, cMo;
  KeyframeOmni kf_omni;
  cv::Mat kf_I, kf_depth, kf_grad, kf_gradient;
  cv::Mat pointSet;

  cv::Mat edges;
  cv::Mat blurred;
  Visualization visualizor_;

  cv::Point minP, maxP;
  double mMin, mMax;
  PointCloud::Ptr pc_;
  bool psFlag;

  int renderCtr=0;
  bool renderFlag = false;

  cv_bridge::CvImagePtr cv_ptr_ps;
  int edge_image_counter = 0; 

  vpHomogeneousMatrix smallShift;

    vpHomogeneousMatrix delta_i;
  void rerender();
};
template <class Scalar, int M, int Options = 0>
using Vector = Eigen::Matrix<Scalar, M, 1, Options>;
template <class Scalar, int Options = 0>
using Vector2 = Vector<Scalar, 2, Options>;
using Vector2d = Vector2<double>;
}


#endif //ESVO_CORE_TRACKING_H

