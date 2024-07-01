#include <esvo_time_surface/TimeSurface.h>
#include <esvo_time_surface/TicToc.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <std_msgs/Float32.h>
#include <glog/logging.h>
#include <thread>

// #define ESVO_TS_LOG

namespace esvo_time_surface 
{
TimeSurface::TimeSurface(ros::NodeHandle & nh, ros::NodeHandle nh_private)
  : nh_(nh)
{
  // setup subscribers and publishers
  
  event_sub_ = nh_.subscribe("events", 0, &TimeSurface::eventsCallback, this);
  // camera_info_sub_ = nh_.subscribe("camera_info", 1, &TimeSurface::cameraInfoCallback_prophesee, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &TimeSurface::cameraInfoCallback, this);
  sync_topic_ = nh_.subscribe("sync", 1, &TimeSurface::syncCallback, this);
  image_transport::ImageTransport it_(nh_);
  time_surface_pub_ = it_.advertise("time_surface", 1);
  pointSet_pub_ = it_.advertise("point_set", 1);

  // parameters
  nh_private.param<bool>("use_sim_time", bUse_Sim_Time_, true);
  nh_private.param<int>("start_time_sec", startTimeSec_, 0);
  nh_private.param<int>("start_time_nsec", startTimeNsec_, 0);
  nh_private.param<bool>("ignore_polarity", ignore_polarity_, true);
  nh_private.param<double>("decay_ms", decay_ms_, 30);
  int TS_mode;
  nh_private.param<int>("time_surface_mode", TS_mode, 0);
  time_surface_mode_ = (TimeSurfaceMode)TS_mode;
  nh_private.param<int>("median_blur_kernel_size", median_blur_kernel_size_, 1);
  nh_private.param<int>("max_event_queue_len", max_event_queue_length_, 20);
  //
  bCamInfoAvailable_ = false;
  bSensorInitialized_ = false;
  if(pEventQueueMat_)
    pEventQueueMat_->clear();
  sensor_size_ = cv::Size(0,0);
  

  sync_time_ = ros::Time((long int)startTimeSec_, (long int)startTimeNsec_);
  pointSet = cv::imread("/home/yufan/Data/experiments/ESVO/EVS/edgemap.png", 0);
}

TimeSurface::~TimeSurface()
{
  time_surface_pub_.shutdown();
}

void TimeSurface::init(int width, int height)
{
  sensor_size_ = cv::Size(width, height);
  bSensorInitialized_ = true;
  pEventQueueMat_.reset(new EventQueueMat(width, height, max_event_queue_length_));
  ROS_INFO("Sensor size: (%d x %d)", sensor_size_.width, sensor_size_.height);
  representation_SILC_ = cv::Mat::zeros(sensor_size_, CV_8UC1);
}

void TimeSurface::createTimeSurfaceAtTime(const ros::Time& external_sync_time)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // std::cout<<sensor_size_.width<<std::endl;
  if(!bSensorInitialized_ || !bCamInfoAvailable_)
    return;

  // std::cout<<"jajaja"<<std::endl;
  // create exponential-decayed Time Surface map.
  const double decay_sec = decay_ms_ / 1000.0;
  cv::Mat time_surface_map;
  time_surface_map = cv::Mat::zeros(sensor_size_, CV_64F);
  // std::cout<<"jajaja"<<std::endl;
  ros::Time external_time;
  if(bPropheseeUsed_)
    external_time = external_sync_time;
  else
    external_time = external_sync_time;
  // Loop through all coordinates
  int r_ = 3;
  int SILC_bound_ = (2*r_+1)*(2*r_+1);
  int k_tos_ = 3;
  int T_tos_ = 241;
  // for(int y=0; y<sensor_size_.height; ++y)
  // {
  //   for(int x=0; x<sensor_size_.width; ++x)
  //   {
  //     dvs_msgs::Event most_recent_event_at_coordXY_before_T;
  //     // std::cout<<y <<" "<<x<<std::endl;
  //     if(pEventQueueMat_->getMostRecentEventBeforeT(x, y, external_time, &most_recent_event_at_coordXY_before_T))
  //     {
  //       const ros::Time& most_recent_stamp_at_coordXY = most_recent_event_at_coordXY_before_T.ts;
  //       // std::cout<<"The time is "<<most_recent_stamp_at_coordXY.toSec()<<std::endl;
  //       if(most_recent_stamp_at_coordXY.toSec() > 0)
  //       {
  //         const double dt = (external_time - most_recent_stamp_at_coordXY).toSec();
  //         // std::cout<<"Dt is "<<dt<<std::endl;
  //         double polarity = (most_recent_event_at_coordXY_before_T.polarity) ? 1.0 : -1.0;
  //         double expVal = std::exp(-dt / decay_sec);
  //         // double expVal = std::exp(0 / decay_sec);
  //         if(!ignore_polarity_)
  //           expVal *= polarity;

  //         // Backward version
  //         if(time_surface_mode_ == BACKWARD)
  //           time_surface_map.at<double>(y,x) = expVal;

  //         // Forward version
  //         if(time_surface_mode_ == FORWARD && bCamInfoAvailable_)
  //         {
  //           Eigen::Matrix<double, 2, 1> uv_rect = precomputed_rectified_points_.block<2, 1>(0, y * sensor_size_.width + x);
  //           size_t u_i, v_i;
  //           if(uv_rect(0) >= 0 && uv_rect(1) >= 0)
  //           {
  //             u_i = std::floor(uv_rect(0));
  //             v_i = std::floor(uv_rect(1));

  //             if(u_i + 1 < sensor_size_.width && v_i + 1 < sensor_size_.height)
  //             {
  //               double fu = uv_rect(0) - u_i;
  //               double fv = uv_rect(1) - v_i;
  //               double fu1 = 1.0 - fu;
  //               double fv1 = 1.0 - fv;
  //               time_surface_map.at<double>(v_i, u_i) += fu1 * fv1 * expVal;
  //               time_surface_map.at<double>(v_i, u_i + 1) += fu * fv1 * expVal;
  //               time_surface_map.at<double>(v_i + 1, u_i) += fu1 * fv * expVal;
  //               time_surface_map.at<double>(v_i + 1, u_i + 1) += fu * fv * expVal;

  //               if(time_surface_map.at<double>(v_i, u_i) > 1)
  //                 time_surface_map.at<double>(v_i, u_i) = 1;
  //               if(time_surface_map.at<double>(v_i, u_i + 1) > 1)
  //                 time_surface_map.at<double>(v_i, u_i + 1) = 1;
  //               if(time_surface_map.at<double>(v_i + 1, u_i) > 1)
  //                 time_surface_map.at<double>(v_i + 1, u_i) = 1;
  //               if(time_surface_map.at<double>(v_i + 1, u_i + 1) > 1)
  //                 time_surface_map.at<double>(v_i + 1, u_i + 1) = 1;
  //             }
  //           }
  //         } // forward
  //       }
  //     } // a most recent event is available
  //     // std::cout<<pointSet.at<unsigned char>(y,x)<<std::endl;

  //   }// loop x
  // }// loop y

//   auto it = InvolvedEvents_.begin();
//     for(;it != InvolvedEvents_.end();it++)
//     {
//       dvs_msgs::Event e = *it;
//       for(int dx = -k_tos_; dx <= k_tos_; dx++)
//         for(int dy = -k_tos_; dy <= k_tos_; dy++)
//         {
//           if(e.x + dx < 0 || e.x + dx >= sensor_size_.width || e.y + dy < 0 || e.y + dy >= sensor_size_.height)
//             continue;
// //          if(e.polarity < 0)
// //            continue;
//           if(representation_SILC_.at<uchar>(e.y+dy, e.x+dx) >= 1)
//             representation_SILC_.at<uchar>(e.y+dy, e.x+dx)--;
// //            representation_TOS_.at<uchar>(e.y+dy, e.x+dx) = representation_TOS_.at<uchar>(e.y+dy, e.x+dx) - substraction_delta_;
//           if(representation_SILC_.at<uchar>(e.y+dy, e.x+dx) < T_tos_)
//             representation_SILC_.at<uchar>(e.y+dy, e.x+dx) = 0;
//         }
//       representation_SILC_.at<uchar>(e.y, e.x) = 255;
//     }
    representation_SILC_.setTo(cv::Scalar(0));
    for(size_t y = 0; y < sensor_size_.height; y++)
      for(size_t x = 0; x < sensor_size_.width; x++)
      {
        for(int dx = -r_; dx <= r_; dx++)
          for(int dy = -r_; dy <= r_; dy++)
          {
            if(x + dx < 0 || x + dx >= sensor_size_.width || y + dy < 0 || y + dy >= sensor_size_.height)
              continue;
            dvs_msgs::Event ev, ev_nb;
            if(!pEventQueueMat_->getMostRecentEventBeforeT(x, y, external_sync_time, &ev) ||
              !pEventQueueMat_->getMostRecentEventBeforeT(x+dx, y+dy, external_sync_time, &ev_nb))
              continue;
            if(ev.ts.toSec() < ev_nb.ts.toSec())
              representation_SILC_.at<uchar>(y,x)++;
          }
      }

  // // polarity
  // if(!ignore_polarity_)
  //   time_surface_map = 255.0 * (time_surface_map + 1.0) / 2.0;
  // else
  //   time_surface_map = 255.0 * time_surface_map;
  // time_surface_map.convertTo(time_surface_map, CV_8U);
  // // pointSet.convertTo(pointSet, CV_8U);

  // // median blur
  // if(median_blur_kernel_size_ > 0)
  //   cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);
  // // Publish event image
  static cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";
  // cv_image.image = time_surface_map.clone();
    // cv::Mat SILC_img = cv::Mat::zeros(sensor_size_, CV_64F);
    // // Add a lock here
    // SILC_img = 255.0 * representation_SILC_ / SILC_bound_;
    // SILC_img.convertTo(SILC_img, CV_8U);
    cv_image.image = representation_SILC_.clone();
  // std::cout<<SILC_img<<std::endl;

  // cv::imwrite("/home/yufan/Data/experiments/ESVO/TS.png", time_surface_map);
  // if(time_surface_mode_ == FORWARD && time_surface_pub_.getNumSubscribers() > 0)
  // {
    cv_image.header.stamp = external_sync_time;
    time_surface_pub_.publish(cv_image.toImageMsg());

  // std::cout<<"here lala"<<std::endl;
  // }
  // if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0)
  // {
  //   cv_bridge::CvImage cv_image2, cv_image3;
  //   cv_image2.encoding = cv_image.encoding;
  //   cv_image2.header.stamp = external_sync_time;
  //   cv::remap(cv_image.image, cv_image2.image, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);
  //   time_surface_pub_.publish(cv_image2.toImageMsg());
  //   cv_image3.encoding = cv_image.encoding;
  //   cv_image3.header.stamp = external_sync_time;
  //   cv_image3.image = pointSet.clone();
  //   pointSet_pub_.publish(cv_image3.toImageMsg());
  //   // std::cout<<time_surface_map<<std::endl;
  // // std::cout<<pointSet.type()<<std::endl;
  // }
  // cv::waitKey(0);
  //yufan added 

  // if(time_surface_mode_ == BACKWARD && bPropheseeUsed_)
  // {
  //   cv_image.header.stamp = external_sync_time;
  //   time_surface_pub_.publish(cv_image.toImageMsg());
  // }
  InvolvedEvents_.clear();
}

void TimeSurface::createTimeSurfaceAtTime_hyperthread(const ros::Time& external_sync_time)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if(!bSensorInitialized_ || !bCamInfoAvailable_)
    return;

  // create exponential-decayed Time Surface map.
  const double decay_sec = decay_ms_ / 1000.0;
  cv::Mat time_surface_map;
  time_surface_map = cv::Mat::zeros(sensor_size_, CV_64F);

  // distribute jobs
  std::vector<Job> jobs(NUM_THREAD_TS);
  size_t num_col_per_thread = sensor_size_.width / NUM_THREAD_TS;
  size_t res_col = sensor_size_.width % NUM_THREAD_TS;
  for(size_t i = 0; i < NUM_THREAD_TS; i++)
  {
    jobs[i].i_thread_ = i;
    jobs[i].pEventQueueMat_ = pEventQueueMat_.get();
    jobs[i].pTimeSurface_ = &time_surface_map;
    jobs[i].start_col_ = num_col_per_thread * i;
    if(i == NUM_THREAD_TS - 1)
      jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1 + res_col;
    else
      jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1;
    jobs[i].start_row_ = 0;
    jobs[i].end_row_ = sensor_size_.height - 1;
    jobs[i].external_sync_time_ = external_sync_time;
    jobs[i].decay_sec_ = decay_sec;
  }

  // hyper thread processing
  std::vector<std::thread> threads;
  threads.reserve(NUM_THREAD_TS);
  for(size_t i = 0; i < NUM_THREAD_TS; i++)
    threads.emplace_back(std::bind(&TimeSurface::thread, this, jobs[i]));
  for(auto& thread:threads)
    if(thread.joinable())
      thread.join();

  // polarity
  if(!ignore_polarity_)
    time_surface_map = 255.0 * (time_surface_map + 1.0) / 2.0;
  else
    time_surface_map = 255.0 * time_surface_map;
  time_surface_map.convertTo(time_surface_map, CV_8U);

  // median blur
  if(median_blur_kernel_size_ > 0)
    cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);

  // Publish event image
  static cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";
  cv_image.image = time_surface_map.clone();

  if(time_surface_mode_ == FORWARD && time_surface_pub_.getNumSubscribers() > 0)
  {
    cv_image.header.stamp = external_sync_time;
    time_surface_pub_.publish(cv_image.toImageMsg());
  }

  if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image2, cv_image3;
    cv_image2.encoding = cv_image.encoding;
    cv_image2.header.stamp = external_sync_time;
    cv::remap(cv_image.image, cv_image2.image, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);
    time_surface_pub_.publish(cv_image2.toImageMsg());
    cv_image3.encoding = cv_image.encoding;
    cv_image3.header.stamp = external_sync_time;
    cv_image3.image = pointSet.clone();
    pointSet_pub_.publish(cv_image3.toImageMsg());
  }
}

void TimeSurface::thread(Job &job)
{
  EventQueueMat & eqMat = *job.pEventQueueMat_;
  cv::Mat& time_surface_map = *job.pTimeSurface_;
  size_t start_col = job.start_col_;
  size_t end_col = job.end_col_;
  size_t start_row = job.start_row_;
  size_t end_row = job.end_row_;
  size_t i_thread = job.i_thread_;

  for(size_t y = start_row; y <= end_row; y++)
    for(size_t x = start_col; x <= end_col; x++)
    {
      dvs_msgs::Event most_recent_event_at_coordXY_before_T;
      if(pEventQueueMat_->getMostRecentEventBeforeT(x, y, job.external_sync_time_, &most_recent_event_at_coordXY_before_T))
      {
        const ros::Time& most_recent_stamp_at_coordXY = most_recent_event_at_coordXY_before_T.ts;
        if(most_recent_stamp_at_coordXY.toSec() > 0)
        {
          const double dt = (job.external_sync_time_ - most_recent_stamp_at_coordXY).toSec();
          double polarity = (most_recent_event_at_coordXY_before_T.polarity) ? 1.0 : -1.0;
          double expVal = std::exp(-dt / job.decay_sec_);
          if(!ignore_polarity_)
            expVal *= polarity;

          // Backward version
          if(time_surface_mode_ == BACKWARD)
            time_surface_map.at<double>(y,x) = expVal;

          // Forward version
          if(time_surface_mode_ == FORWARD && bCamInfoAvailable_)
          {
            Eigen::Matrix<double, 2, 1> uv_rect = precomputed_rectified_points_.block<2, 1>(0, y * sensor_size_.width + x);
            size_t u_i, v_i;
            if(uv_rect(0) >= 0 && uv_rect(1) >= 0)
            {
              u_i = std::floor(uv_rect(0));
              v_i = std::floor(uv_rect(1));

              if(u_i + 1 < sensor_size_.width && v_i + 1 < sensor_size_.height)
              {
                double fu = uv_rect(0) - u_i;
                double fv = uv_rect(1) - v_i;
                double fu1 = 1.0 - fu;
                double fv1 = 1.0 - fv;
                time_surface_map.at<double>(v_i, u_i) += fu1 * fv1 * expVal;
                time_surface_map.at<double>(v_i, u_i + 1) += fu * fv1 * expVal;
                time_surface_map.at<double>(v_i + 1, u_i) += fu1 * fv * expVal;
                time_surface_map.at<double>(v_i + 1, u_i + 1) += fu * fv * expVal;

                if(time_surface_map.at<double>(v_i, u_i) > 1)
                  time_surface_map.at<double>(v_i, u_i) = 1;
                if(time_surface_map.at<double>(v_i, u_i + 1) > 1)
                  time_surface_map.at<double>(v_i, u_i + 1) = 1;
                if(time_surface_map.at<double>(v_i + 1, u_i) > 1)
                  time_surface_map.at<double>(v_i + 1, u_i) = 1;
                if(time_surface_map.at<double>(v_i + 1, u_i + 1) > 1)
                  time_surface_map.at<double>(v_i + 1, u_i + 1) = 1;
              }
            }
          } // forward
        }
      } // a most recent event is available
    }
}

void TimeSurface::syncCallback(const std_msgs::TimeConstPtr& msg)
{
  // if(bUse_Sim_Time_)
  //   sync_time_ = ros::Time::now();
  //   // std::cout<<"jIAJIAJIA"<<std::endl;
  // else
  //   sync_time_ = msg->data;

  if(events_.size() < 1)
    return;
  // how many events per second 
  // evt_ctr ++;
  // if(evt_ctr == 99){
  //   evt_ctr = 0;
  //   std::cout<<"Event in 1s = "<<events_.size() - evt_persec<<std::endl;
  //   evt_persec = events_.size();
  // }
  // if((events_.back().ts - sync_time_).toSec() < 0)
  //   return;
  // ros::Duration delta_t(0.01);
  // sync_time_ = sync_time_ + delta_t;
  sync_time_ = events_.back().ts;
  // std::cout<<"Last event time is      "<<events_.back().ts<<std::endl;
  // std::cout<<"Sync now time is        "<<sync_time_<<std::endl;
  // ros::Time tmp = ros::Time((long int)startTimeSec_, (long int)startTimeNsec_);
  // std::cout<<"sync time = "<<sync_time_<<std::endl;
  // std::cout<<tmp<<std::endl;
  // std::cout<<" "<<std::endl;
#ifdef ESVO_TS_LOG
    TicToc tt;
    tt.tic();
#endif
    if(NUM_THREAD_TS == 1)
      createTimeSurfaceAtTime(sync_time_);
    if(NUM_THREAD_TS > 1)
      createTimeSurfaceAtTime_hyperthread(sync_time_);
#ifdef ESVO_TS_LOG
    LOG(INFO) << "Time Surface map's creation takes: " << tt.toc() << " ms.";
#endif
}

void TimeSurface::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if(bCamInfoAvailable_)
    return;

  cv::Size sensor_size(msg->width, msg->height);
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i+j*3];

  distortion_model_ = msg->distortion_model;
  // std::cout<<msg->distortion_model<<std::endl;
  // std::cout<<"width and height "<<msg->width<<" "<<msg->height<<std::endl;
  dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    dist_coeffs_.at<double>(i) = msg->D[i];

  rectification_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      rectification_matrix_.at<double>(cv::Point(i, j)) = msg->R[i+j*3];

  projection_matrix_ = cv::Mat(3, 4, CV_64F);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      projection_matrix_.at<double>(cv::Point(i, j)) = msg->P[i+j*4];
  // std::cout<<"Dis model is "<<distortion_model_<<std::endl;
  if(distortion_model_ == "equidistant")
  {
    cv::fisheye::initUndistortRectifyMap(camera_matrix_, dist_coeffs_,
                                         rectification_matrix_, projection_matrix_,
                                         sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
    bCamInfoAvailable_ = true;
    ROS_INFO("Camera information is loaded (Distortion model %s).", distortion_model_.c_str());
  }
  else if(distortion_model_ == "plumb_bob")
  {
    cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_,
                                rectification_matrix_, projection_matrix_,
                                sensor_size, CV_32FC1, undistort_map1_, undistort_map2_);
    bCamInfoAvailable_ = true;
    bPropheseeUsed_ = true;
    ROS_INFO("Camera information is loaded (Distortion model %s).", distortion_model_.c_str());
  }
  else
  {
    ROS_ERROR_ONCE("Distortion model here %s is not supported.", distortion_model_.c_str());
    bCamInfoAvailable_ = false;
    return;
  }

  /* pre-compute the undistorted-rectified look-up table */
  precomputed_rectified_points_ = Eigen::Matrix2Xd(2, sensor_size.height * sensor_size.width);
  // raw coordinates
  cv::Mat_<cv::Point2f> RawCoordinates(1, sensor_size.height * sensor_size.width);
  for (int y = 0; y < sensor_size.height; y++)
  {
    for (int x = 0; x < sensor_size.width; x++)
    {
      int index = y * sensor_size.width + x;
      RawCoordinates(index) = cv::Point2f((float) x, (float) y);
    }
  }
  // undistorted-rectified coordinates
  cv::Mat_<cv::Point2f> RectCoordinates(1, sensor_size.height * sensor_size.width);
  if (distortion_model_ == "plumb_bob")
  {
    cv::undistortPoints(RawCoordinates, RectCoordinates, camera_matrix_, dist_coeffs_,
                        rectification_matrix_, projection_matrix_);
    ROS_INFO("Undistorted-Rectified Look-Up Table with Distortion model: %s", distortion_model_.c_str());
  }
  else if (distortion_model_ == "equidistant")
  {
    cv::fisheye::undistortPoints(
      RawCoordinates, RectCoordinates, camera_matrix_, dist_coeffs_,
      rectification_matrix_, projection_matrix_);
    ROS_INFO("Undistorted-Rectified Look-Up Table with Distortion model: %s", distortion_model_.c_str());
  }
  else
  {
    std::cout << "Unknown distortion model is provided." << std::endl;
    exit(-1);
  }
  // load look-up table
  for (size_t i = 0; i < sensor_size.height * sensor_size.width; i++)
  {
    precomputed_rectified_points_.col(i) = Eigen::Matrix<double, 2, 1>(
      RectCoordinates(i).x, RectCoordinates(i).y);
  }
  ROS_INFO("Undistorted-Rectified Look-Up Table has been computed.");
}

//modified from 
void TimeSurface::cameraInfoCallback_prophesee(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if(bCamInfoAvailable_)
    return;

  cv::Size sensor_size(msg->width, msg->height);
  bCamInfoAvailable_ = true;
  bPropheseeUsed_ = true;
  ROS_INFO("Undistorted-Rectified Look-Up Table has been computed.");
}


void TimeSurface::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // std::cout<<"bulubulu"<<std::endl;
  std::lock_guard<std::mutex> lock(data_mutex_);

  if(!bSensorInitialized_)
    init(msg->width, msg->height);

  for(const dvs_msgs::Event& e : msg->events)
  {
    // std::cout<<"Time diff is "<<ros::Time::now().toSec() - e.ts.toSec()<<std::endl;
    // std::cout<<"Event time is "<< e.ts.toSec()<<"Current time is "<< ros::Time::now().toSec()<<std::endl;
    events_.push_back(e);
    InvolvedEvents_.push_back(e);
    int i = events_.size() - 2;
    while(i >= 0 && events_[i].ts > e.ts)
    {
      events_[i+1] = events_[i];
      i--;
    }
    events_[i+1] = e;

    const dvs_msgs::Event& last_event = events_.back();
    // Use a vector instead
    pEventQueueMat_->insertEvent(last_event);
  }

  // dvs_msgs::Event& ttevent = events_.back();
  // std::cout<<"Last event timestamp is "<<events_.back().ts<<std::endl;
  // std::cout<<"Sync now time is        "<<sync_time_<<std::endl;
  // std::cout<<"Event queue size is "<<events_.size()<<std::endl;
  clearEventQueue();

  //Yufan added for debug
  // sync_time_ = ttevent.ts;
  // if(NUM_THREAD_TS == 1)
  //   createTimeSurfaceAtTime(sync_time_);

}

void TimeSurface::clearEventQueue()
{
  static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 5000000;
  if (events_.size() > MAX_EVENT_QUEUE_LENGTH)
  {
    size_t remove_events = events_.size() - MAX_EVENT_QUEUE_LENGTH;
    events_.erase(events_.begin(), events_.begin() + remove_events);
  }
}

} // namespace esvo_time_surface