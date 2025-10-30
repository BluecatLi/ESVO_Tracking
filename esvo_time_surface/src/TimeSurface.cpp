#include <esvo_time_surface/TimeSurface.h>
#include <esvo_time_surface/TicToc.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <std_msgs/Float32.h>
#include <glog/logging.h>
#include <thread>
// #include <vector>

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
  nh_private.param<int>("event_accumulate_number", eventNumber_, 500);
  int TS_mode;
  nh_private.param<int>("time_surface_mode", TS_mode, 0);
  time_surface_mode_ = (TimeSurfaceMode)TS_mode;
  nh_private.param<int>("median_blur_kernel_size", median_blur_kernel_size_, 1);
  nh_private.param<int>("max_event_queue_len", max_event_queue_length_, 20);
  //
  bCamInfoAvailable_ = false;
  bSensorInitialized_ = false;
  // if(pEventQueueMat_)
  //   pEventQueueMat_->clear();
  sensor_size_ = cv::Size(0,0);
  

  sync_time_ = ros::Time((long int)startTimeSec_, (long int)startTimeNsec_);
  // pointSet = cv::imread("/home/yufan/Data/experiments/ESVO/EVS/edgemap_box.png", 0);
  // pointSet = cv::imread("/home/yufan/Data/2025/0318/pointset_Cupnoodles.png", 0);
  // pointSet = cv::imread("/home/yufan/Data/2025/0405/pointset_Cupnoodles.png", 0);
  pointSet = cv::imread("/home/yufan/Data/2025/0212/ps.png", 0);
  // pointSet = cv::imread("/home/yufan/Data/2024/1018/ev_resized.png", 0);
  prev_TS = cv::Mat::zeros(sensor_size_, CV_8UC1);
  cv::glob("/home/yufan/Data/E-POSE/wrench/t_10_gl/*_segmented_image.png", seg_paths_, /*recursive=*/false);
  dvs_msgs::EventArrayPtr msg(new dvs_msgs::EventArray);
  const std::string CSV = "/home/yufan/Data/E-POSE/block/t_10_gl_events.csv";
  const int WIDTH  = 346;   // or 640
  const int HEIGHT = 260;   // or 480

  if (!loadEventsCSV(CSV, WIDTH, HEIGHT, *msg)) {
      ROS_ERROR("Failed to load events CSV");
      return;
  }

  // Your existing processing:
  for (const dvs_msgs::Event& e : msg->events) {
      events_.push_back(e);
      const dvs_msgs::Event& last_event = events_.back();
      // ensure pEventTs_ is sized to width*height and initialized (e.g., zeros)
      pEventTs_[last_event.x + last_event.y * msg->width] = last_event.ts.toSec();
  }

  clearEventQueue();
  std::cout<<"Loaded "<< events_.size()<<" events!"<<std::endl;
}

TimeSurface::~TimeSurface()
{
  time_surface_pub_.shutdown();
}

void TimeSurface::init(int width, int height)
{
  sensor_size_ = cv::Size(width, height);
  bSensorInitialized_ = true;
  // pEventQueueMat_.reset(new EventQueueMat(width, height, max_event_queue_length_));
  ROS_INFO("Sensor size: (%d x %d)", sensor_size_.width, sensor_size_.height);
  representation_SILC_ = cv::Mat::zeros(sensor_size_, CV_8UC1);
}
bool TimeSurface::loadEventsCSV(const std::string& csv_path, int width, int height, dvs_msgs::EventArray& out)
{
    std::ifstream f(csv_path);
    if (!f.is_open()) {
        ROS_ERROR_STREAM("Failed to open CSV: " << csv_path);
        return false;
    }

    out.events.clear();
    out.height = height;
    out.width  = width;
    out.header.stamp = ros::Time(0);
    out.header.frame_id = "prophesee_camera";  // set as you like

    std::string line;
    // Skip header
    if (!std::getline(f, line)) {
        ROS_ERROR("CSV empty");
        return false;
    }

    out.events.reserve(1 << 20); // optional: reserve some space if large

    size_t bad = 0;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string s;
        int x=0, y=0, p=0;
        double t=0.0;

        // Columns: x,y,t,p,file,idx_in_file
        if (!std::getline(ss, s, ',')) { bad++; continue; } x = std::stoi(s);
        if (!std::getline(ss, s, ',')) { bad++; continue; } y = std::stoi(s);
        if (!std::getline(ss, s, ',')) { bad++; continue; } t = std::stod(s);
        if (!std::getline(ss, s, ',')) { bad++; continue; } p = std::stoi(s);

        // Skip the rest of the columns quickly
        // (file, idx_in_file) -> consume but ignore
        std::getline(ss, s, ','); // file
        std::getline(ss, s, ','); // idx

        // Bounds check (optional, but safer)
        if (x < 0 || x >= width || y < 0 || y >= height) continue;

        dvs_msgs::Event e;
        e.x = x;
        e.y = y;
        e.ts = ros::Time().fromSec(t);
        e.polarity = (p > 0);

        out.events.push_back(e);
    }

    if (bad) {
        ROS_WARN_STREAM("Skipped " << bad << " malformed CSV rows.");
    }
    return true;
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
  for(int y=0; y<sensor_size_.height; ++y)
  {
    for(int x=0; x<sensor_size_.width; ++x)
    {
      if(pEventTs_[x + y*sensor_size_.width] == 0)
        continue;
      const double dt = external_time.toSec() - pEventTs_[x + y*sensor_size_.width];
      double expVal = std::exp(-dt / decay_sec);
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


      // dvs_msgs::Event most_recent_event_at_coordXY_before_T;
      // // std::cout<<y <<" "<<x<<std::endl;
      // if(pEventQueueMat_->getMostRecentEventBeforeT(x, y, external_time, &most_recent_event_at_coordXY_before_T))
      // {
      //   const ros::Time& most_recent_stamp_at_coordXY = most_recent_event_at_coordXY_before_T.ts;
      //   // std::cout<<"The time is "<<most_recent_stamp_at_coordXY.toSec()<<std::endl;
      //   if(most_recent_stamp_at_coordXY.toSec() > 0)
      //   {
      //     const double dt = (external_time - most_recent_stamp_at_coordXY).toSec();
      //     // std::cout<<"Dt is "<<dt<<std::endl;
      //     double polarity = (most_recent_event_at_coordXY_before_T.polarity) ? 1.0 : -1.0;
      //     double expVal = std::exp(-dt / decay_sec);
      //     // double expVal = std::exp(0 / decay_sec);
      //     if(!ignore_polarity_)
      //       expVal *= polarity;

      //     // Backward version
      //     if(time_surface_mode_ == BACKWARD)
      //       time_surface_map.at<double>(y,x) = expVal;

      //     // Forward version
      //     if(time_surface_mode_ == FORWARD && bCamInfoAvailable_)
      //     {
      //       Eigen::Matrix<double, 2, 1> uv_rect = precomputed_rectified_points_.block<2, 1>(0, y * sensor_size_.width + x);
      //       size_t u_i, v_i;
      //       if(uv_rect(0) >= 0 && uv_rect(1) >= 0)
      //       {
      //         u_i = std::floor(uv_rect(0));
      //         v_i = std::floor(uv_rect(1));

      //         if(u_i + 1 < sensor_size_.width && v_i + 1 < sensor_size_.height)
      //         {
      //           double fu = uv_rect(0) - u_i;
      //           double fv = uv_rect(1) - v_i;
      //           double fu1 = 1.0 - fu;
      //           double fv1 = 1.0 - fv;
      //           time_surface_map.at<double>(v_i, u_i) += fu1 * fv1 * expVal;
      //           time_surface_map.at<double>(v_i, u_i + 1) += fu * fv1 * expVal;
      //           time_surface_map.at<double>(v_i + 1, u_i) += fu1 * fv * expVal;
      //           time_surface_map.at<double>(v_i + 1, u_i + 1) += fu * fv * expVal;

      //           if(time_surface_map.at<double>(v_i, u_i) > 1)
      //             time_surface_map.at<double>(v_i, u_i) = 1;
      //           if(time_surface_map.at<double>(v_i, u_i + 1) > 1)
      //             time_surface_map.at<double>(v_i, u_i + 1) = 1;
      //           if(time_surface_map.at<double>(v_i + 1, u_i) > 1)
      //             time_surface_map.at<double>(v_i + 1, u_i) = 1;
      //           if(time_surface_map.at<double>(v_i + 1, u_i + 1) > 1)
      //             time_surface_map.at<double>(v_i + 1, u_i + 1) = 1;
      //         }
      //       }
      //     } // forward
      //   }
      // } // a most recent event is available
      // std::cout<<pointSet.at<unsigned char>(y,x)<<std::endl;

    }// loop x
  }// loop y

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
    // representation_SILC_.setTo(cv::Scalar(0));
    // for(size_t y = 0; y < sensor_size_.height; y++)
    //   for(size_t x = 0; x < sensor_size_.width; x++)
    //   {
    //     for(int dx = -r_; dx <= r_; dx++)
    //       for(int dy = -r_; dy <= r_; dy++)
    //       {
    //         if(x + dx < 0 || x + dx >= sensor_size_.width || y + dy < 0 || y + dy >= sensor_size_.height)
    //           continue;
    //         dvs_msgs::Event ev, ev_nb;
    //         if(!pEventQueueMat_->getMostRecentEventBeforeT(x, y, external_sync_time, &ev) ||
    //           !pEventQueueMat_->getMostRecentEventBeforeT(x+dx, y+dy, external_sync_time, &ev_nb))
    //           continue;
    //         if(ev.ts.toSec() < ev_nb.ts.toSec())
    //           representation_SILC_.at<uchar>(y,x)++;
    //       }
    //   }

  // polarity
  if(!ignore_polarity_)
    time_surface_map = 255.0 * (time_surface_map + 1.0) / 2.0;
  else
    time_surface_map = 255.0 * time_surface_map;
  time_surface_map.convertTo(time_surface_map, CV_8U);
  // pointSet.convertTo(pointSet, CV_8U);

  // // median blur
  // if(median_blur_kernel_size_ > 0)
  //   cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);
  // // Publish event image
  static cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";
  cv_image.image = time_surface_map.clone();
    // cv::Mat SILC_img = cv::Mat::zeros(sensor_size_, CV_64F);
    // // Add a lock here
    // SILC_img = 255.0 * representation_SILC_ / SILC_bound_;
    // SILC_img.convertTo(SILC_img, CV_8U);
    // cv_image.image = representation_SILC_.clone();
  // std::cout<<SILC_img<<std::endl;

  cv::imwrite("/home/yufan/Data/experiments/ESVO/TS.png", time_surface_map);
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
  // InvolvedEvents_.clear();
}

void TimeSurface::createTimeSurfaceAtTime_hyperthread(const ros::Time& external_sync_time)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if(!bSensorInitialized_ || !bCamInfoAvailable_)
    return;

  // create exponential-decayed Time Surface map.
  const double decay_sec = decay_ms_  / 1000.0;
  cv::Mat time_surface_map;
  time_surface_map = cv::Mat::zeros(sensor_size_, CV_64F);

  // distribute jobs
  std::vector<Job> jobs(NUM_THREAD_TS);
  size_t num_col_per_thread = sensor_size_.width / NUM_THREAD_TS;
  size_t res_col = sensor_size_.width % NUM_THREAD_TS;
  for(size_t i = 0; i < NUM_THREAD_TS; i++)
  {
    jobs[i].i_thread_ = i;
    // jobs[i].pEventQueueMat_ = pEventQueueMat_.get();
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
    // time_surface_map = 255.0 * (1-time_surface_map);
    time_surface_map = 255.0 * time_surface_map;
  time_surface_map.convertTo(time_surface_map, CV_8U);



  // cv::imwrite("/home/yufan/Data/2024/1208/TS.png", time_surface_map);  
  cv::Mat outputImage;
  int kernelSize = 3; // Kernel size must be odd and greater than 1 (e.g., 3, 5, 7)
    cv::medianBlur(time_surface_map, outputImage, kernelSize);
  // cv::imwrite("/home/yufan/Data/2024/1208/TSMedian.png", outputImage);  
  cv::Mat resizedTS;
  cv::resize(outputImage, resizedTS, cv::Size(160, 90), 0, 0, cv::INTER_AREA);
  cv::threshold(resizedTS, resizedTS, 1, 255, cv::THRESH_BINARY);
  // cv::imwrite("/home/yufan/Data/2024/1208/TSResized.png", resizedTS);
  // cv::waitKey(0);
  // median blur
  if(median_blur_kernel_size_ > 0)
    cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);

  // Publish event image
  static cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";

  if(time_surface_mode_ == FORWARD && time_surface_pub_.getNumSubscribers() > 0)
  {
    cv_image.header.stamp = external_sync_time;
    time_surface_pub_.publish(cv_image.toImageMsg());
  }
    //   if (pointSet.type() != CV_64F || time_surface_map.type() != CV_64F) {
    //     std::cerr << "Matrices must be of type CV_64F (double)." << std::endl;
    //     return;
    // }

    // // Ensure the size matches the sensor size
    // if (pointSet.size() != sensor_size_ || time_surface_map.size() != sensor_size_) {
    //     std::cerr << "Matrix size does not match sensor size." << std::endl;
    //     return;
    // }

  //   uchar depth = pointSet.type() & CV_MAT_DEPTH_MASK;

  //   std::string r;
  //     switch (depth) {
  //     case CV_8U:  r = "8U"; break;
  //     case CV_8S:  r = "8S"; break;
  //     case CV_16U: r = "16U"; break;
  //     case CV_16S: r = "16S"; break;
  //     case CV_32S: r = "32S"; break;
  //     case CV_32F: r = "32F"; break;
  //     case CV_64F: r = "64F"; break;
  //     default:     r = "User"; break;
  // }
  //   uchar chans = 1 + (pointSet.type() >> CV_CN_SHIFT);
  //   r += "C";
  //   r += (chans + '0');
  // std::cout<<r<<std::endl;


  cv_image.image = time_surface_map.clone();
  // std::cout<<pointSet<<std::endl;
  cv_bridge::CvImage cv_image2, cv_image3;
  cv_image2.encoding = cv_image.encoding;
  cv_image2.header.stamp = external_sync_time;
  cv::remap(cv_image.image, cv_image2.image, undistort_map1_, undistort_map2_, CV_INTER_LINEAR);

  // if (!cv_image2.image.empty() && !prev_TS.empty()) {
  // cv_image2.image.convertTo(cv_image2.image, CV_32F);
  // prev_TS.convertTo(prev_TS, CV_32F);
  // cv_image2.image = 0.5 * cv_image2.image + 0.25 * prev_TS;
  
  // cv::normalize(cv_image2.image, cv_image2.image, 0, 255, cv::NORM_MINMAX);
  // cv_image2.image.convertTo(cv_image2.image, CV_8UC1);
  // prev_TS = cv_image2.image;

  // }
  //Save the TSs with pointset
  cv::Mat ts_mask = cv_image2.image;  

  // cv::Mat resizedTS;
  // cv::resize(ts_mask, resizedTS, cv::Size(160, 90), 0, 0, cv::INTER_AREA);
  // cv::threshold(resizedTS, resizedTS, 2, 255, cv::THRESH_BINARY);
  //   uchar depth = pointSet.type() & CV_MAT_DEPTH_MASK;
  //     std::string r;
  //     switch (depth) {
  //     case CV_8U:  r = "8U"; break;
  //     case CV_8S:  r = "8S"; break;
  //     case CV_16U: r = "16U"; break;
  //     case CV_16S: r = "16S"; break;
  //     case CV_32S: r = "32S"; break;
  //     case CV_32F: r = "32F"; break;
  //     case CV_64F: r = "64F"; break;
  //     default:     r = "User"; break;
  // }
  //   uchar chans = 1 + (ts_mask.type() >> CV_CN_SHIFT);
  //   r += "C";
  //   r += (chans + '0');
  // std::cout<<r<<std::endl;

  // for (int i = 0; i < sensor_size_.height; ++i) {
  //       for (int j = 0; j < sensor_size_.width; ++j) {
  //           if (pointSet.at<uchar>(i, j) != 0) { // Assuming the matrices are of type CV_8U
  //               ts_mask.at<uchar>(i, j) = 255;
  //               // std::cout<<"Bulabula"<<std::endl;
  //           }
  //       }
  //   }
  // std::stringstream ss;
  // ss << std::setw(10) << std::setfill('0') << external_sync_time.sec << "_" << std::setw(9) << std::setfill('0') << external_sync_time.nsec;
  // std::string filename = "/home/yufan/Data/2024/0910/exp1/" + ss.str() + ".png";
  // cv::imwrite(filename, ts_mask);
  int tileWidth = 16;
  int tileHeight = 9;
std::vector<int> nonZeroCounts;
    for (int row = 0; row < 90; row += tileHeight) {
        for (int col = 0; col < 160; col += tileWidth) {
            // Define the region of interest (ROI) for each tile
            cv::Rect tileRect(col, row, tileWidth, tileHeight);
            cv::Mat tile = resizedTS(tileRect);

            // Count the non-zero pixels in the tile
            int nonZeroCount = cv::countNonZero(tile);
            nonZeroCounts.push_back(nonZeroCount);
        }
    }
  
    


  cv::resize(pointSet, pointSet, cv::Size(160, 90), 0, 0, cv::INTER_AREA);
  cv::threshold(pointSet, pointSet, 2, 255, cv::THRESH_BINARY);
  cv::Mat binaryMask;
  cv::threshold(resizedTS, binaryMask, 5, 1, cv::THRESH_BINARY);
  cv::Mat disField;
  cv::distanceTransform(1-binaryMask, disField, cv::DIST_L2, 5);
  for (int y = 0; y < disField.rows; y++) {
        for (int x = 0; x < disField.cols; x++) {
          int tileIndex = (y / tileHeight) * (160 / tileWidth) + x / tileWidth;
          int disPara;
          if(nonZeroCounts[tileIndex] > 20)
            disPara = 2;
          else if (nonZeroCounts[tileIndex] > 10)
            disPara = 4;
          else
            disPara = 6;
            if (disField.at<float>(y, x) > disPara) {
                disField.at<float>(y, x) = 0; // Set to 0 if distance > 10
            }
            else (disField.at<float>(y, x) = disPara - disField.at<float>(y, x));
        }
    }
  cv::normalize(disField, disField, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    for (int i = 0; i < 90; ++i) {
        for (int j = 0; j < 160; ++j) {
            if (resizedTS.at<uchar>(i, j) == 0) { // Assuming the matrices are of type CV_8U
                resizedTS.at<uchar>(i, j) = disField.at<uchar>(i, j);
                // std::cout<<"Bulabula"<<std::endl;
            }
            // if (pointSet.at<uchar>(i, j) != 0) { // Assuming the matrices are of type CV_8U
            //     ts_mask.at<uchar>(i, j) = 255;
            //     std::cout<<"Bulabula"<<std::endl;
            // }
        }
    }
  // std::stringstream ss;
  // ss << std::setw(10) << std::setfill('0') << external_sync_time.sec << "_" << std::setw(9) << std::setfill('0') << external_sync_time.nsec;
  // std::string filename = "/home/yufan/Data/2024/1208/adds.png";
  // cv::imwrite(filename, resizedTS);

  // cv_image2.image =  resizedTS;
  cv_image2.image =  time_surface_map;

  // cv::waitKey(0);
    
  if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0)
  {
    time_surface_pub_.publish(cv_image2.toImageMsg());
    cv_image3.encoding = cv_image.encoding;
    cv_image3.header.stamp = external_sync_time;
    cv_image3.image = pointSet.clone();
    pointSet_pub_.publish(cv_image3.toImageMsg());
  }
}

void TimeSurface::thread(Job &job)
{
  // EventQueueMat & eqMat = *job.pEventQueueMat_;
  cv::Mat& time_surface_map = *job.pTimeSurface_;
  size_t start_col = job.start_col_;
  size_t end_col = job.end_col_;
  size_t start_row = job.start_row_;
  size_t end_row = job.end_row_;
  size_t i_thread = job.i_thread_;

  for(size_t y = start_row; y <= end_row; y++)
    for(size_t x = start_col; x <= end_col; x++)
    {
      if(pEventTs_[x + y*sensor_size_.width] == 0)
        continue;
      const double dt = job.external_sync_time_.toSec() - pEventTs_[x + y*sensor_size_.width];
      // double expVal = std::exp(-dt / job.decay_sec_);
      double expVal;
      if(dt > 0.5)
        expVal = 0;
      else
        expVal = 1;
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
      // dvs_msgs::Event most_recent_event_at_coordXY_before_T;
      // if(pEventQueueMat_->getMostRecentEventBeforeT(x, y, job.external_sync_time_, &most_recent_event_at_coordXY_before_T))
      // {
      //   const ros::Time& most_recent_stamp_at_coordXY = most_recent_event_at_coordXY_before_T.ts;
      //   if(most_recent_stamp_at_coordXY.toSec() > 0)
      //   {
      //     const double dt = (job.external_sync_time_ - most_recent_stamp_at_coordXY).toSec();
      //     double polarity = (most_recent_event_at_coordXY_before_T.polarity) ? 1.0 : -1.0;
      //     double expVal = std::exp(-dt / job.decay_sec_);
      //     if(!ignore_polarity_)
      //       expVal *= polarity;

      //     // Backward version
      //     if(time_surface_mode_ == BACKWARD)
      //       time_surface_map.at<double>(y,x) = expVal;

      //     // Forward version
      //     if(time_surface_mode_ == FORWARD && bCamInfoAvailable_)
      //     {
      //       Eigen::Matrix<double, 2, 1> uv_rect = precomputed_rectified_points_.block<2, 1>(0, y * sensor_size_.width + x);
      //       size_t u_i, v_i;
      //       if(uv_rect(0) >= 0 && uv_rect(1) >= 0)
      //       {
      //         u_i = std::floor(uv_rect(0));
      //         v_i = std::floor(uv_rect(1));

      //         if(u_i + 1 < sensor_size_.width && v_i + 1 < sensor_size_.height)
      //         {
      //           double fu = uv_rect(0) - u_i;
      //           double fv = uv_rect(1) - v_i;
      //           double fu1 = 1.0 - fu;
      //           double fv1 = 1.0 - fv;
      //           time_surface_map.at<double>(v_i, u_i) += fu1 * fv1 * expVal;
      //           time_surface_map.at<double>(v_i, u_i + 1) += fu * fv1 * expVal;
      //           time_surface_map.at<double>(v_i + 1, u_i) += fu1 * fv * expVal;
      //           time_surface_map.at<double>(v_i + 1, u_i + 1) += fu * fv * expVal;

      //           if(time_surface_map.at<double>(v_i, u_i) > 1)
      //             time_surface_map.at<double>(v_i, u_i) = 1;
      //           if(time_surface_map.at<double>(v_i, u_i + 1) > 1)
      //             time_surface_map.at<double>(v_i, u_i + 1) = 1;
      //           if(time_surface_map.at<double>(v_i + 1, u_i) > 1)
      //             time_surface_map.at<double>(v_i + 1, u_i) = 1;
      //           if(time_surface_map.at<double>(v_i + 1, u_i + 1) > 1)
      //             time_surface_map.at<double>(v_i + 1, u_i + 1) = 1;
      //         }
      //       }
      //     } // forward
      //   }
      // } // a most recent event is available
    }
}


void TimeSurface::createEventAccumulation(int N, const ros::Time& external_sync_time)
{
  cv::Mat event_accumulation;
  event_accumulation = cv::Mat::zeros(sensor_size_, CV_64F);
  auto it = events_.rbegin(); // Reverse iterator pointing to the last element
  const dvs_msgs::Event& lastEv = *it;
  for (int i = 0; i < N && it != events_.rend(); ++i, ++it) {
      const dvs_msgs::Event& event = *it;
      // std::cout<<event.y<<" "<<event.x<<std::endl;
      event_accumulation.at<double>(event.y,event.x) = 255;
  }
  const dvs_msgs::Event& firstEv = *it;
  std::cout<<1000*(lastEv.ts-firstEv.ts).toSec()<<std::endl;
  event_accumulation.convertTo(event_accumulation, CV_8U);
  cv::Mat outputImage;
  int kernelSize = 1; // Kernel size must be odd and greater than 1 (e.g., 3, 5, 7)
    cv::medianBlur(event_accumulation, outputImage, kernelSize);
  // cv::Mat temp;
  // cv::Mat disI = cv::Mat::zeros(sensor_size_, CV_64F);
  // assignDistances(outputImage, disI, 10);
  // cv::normalize(disI, outputImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);



  int tileWidth = 64;
  int tileHeight = 48;
  std::vector<int> nonZeroCounts;
  for (int row = 0; row < 480; row += tileHeight) {
      for (int col = 0; col < 640; col += tileWidth) {
          // Define the region of interest (ROI) for each tile
          cv::Rect tileRect(col, row, tileWidth, tileHeight);
          cv::Mat tile = outputImage(tileRect);

          // Count the non-zero pixels in the tile
          int nonZeroCount = cv::countNonZero(tile);
          nonZeroCounts.push_back(nonZeroCount);
      }
  }
  // cv::resize(pointSet, pointSet, cv::Size(160, 120), 0, 0, cv::INTER_AREA);
  // cv::threshold(pointSet, pointSet, 2, 255, cv::THRESH_BINARY);
  cv::Mat binaryMask;
  cv::threshold(outputImage, binaryMask, 5, 1, cv::THRESH_BINARY);
  cv::Mat disField;
  cv::distanceTransform(1-binaryMask, disField, cv::DIST_L2, 5);

  std::sort(nonZeroCounts.begin(), nonZeroCounts.end());

  // // Print the sorted values
  // std::cout << "Sorted values: ";
  // for (int value : nonZeroCounts) {
  //     std::cout << value << " ";
  // }
  // std::cout << std::endl;
  // cv::waitKey(0);
  for (int y = 0; y < disField.rows; y++) {
        for (int x = 0; x < disField.cols; x++) {
          int tileIndex = (y / tileHeight) * (640 / tileWidth) + x / tileWidth;
          int disPara;
          // std::cout<<nonZeroCounts[tileIndex]<<std::endl;
          // if(nonZeroCounts[tileIndex] > 20)
          //   disPara = 4;
          // else if (nonZeroCounts[tileIndex] > 17)
          //   disPara = 8;
          // else if (nonZeroCounts[tileIndex] > 15)
          //   disPara = 12;
          // else 
            disPara = 12;
            if (disField.at<float>(y, x) > disPara) {
                disField.at<float>(y, x) = 0; // Set to 0 if distance > 10
            }
            else (disField.at<float>(y, x) = disPara - disField.at<float>(y, x));
        }
    }
  cv::normalize(disField, disField, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    for (int i = 0; i < 480; ++i) {
        for (int j = 0; j < 640; ++j) {
            if (outputImage.at<uchar>(i, j) == 0) { // Assuming the matrices are of type CV_8U
                outputImage.at<uchar>(i, j) = disField.at<uchar>(i, j);
                // std::cout<<"Bulabula"<<std::endl;
            }

        }
    }

  // // //Save images for debug
  // std::stringstream ss;
  // ss << std::setw(10) << std::setfill('0') << external_sync_time.sec << "_" << std::setw(9) << std::setfill('0') << external_sync_time.nsec;
  // std::string filename = "/home/yufan/Data/2025/0311/exp1/DF3/" + ss.str() + ".png";
  // cv::imwrite(filename, outputImage);

  // cv::imwrite("/home/yufan/Data/2025/0311/ps.png", pointSet);
  
  static cv_bridge::CvImage cv_image, cv_image3;
  cv_image.encoding = "mono8";
  cv_image.image = outputImage.clone();
  cv_image.header.stamp = external_sync_time;
  if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0)
  {
    time_surface_pub_.publish(cv_image.toImageMsg());
    cv_image3.encoding = cv_image.encoding;
    cv_image3.header.stamp = external_sync_time;
    cv_image3.image = pointSet.clone();
    pointSet_pub_.publish(cv_image3.toImageMsg());
  }
}


void TimeSurface::createEventDistanceField(int N, const ros::Time& external_sync_time)
{
  cv::Mat event_accumulation;
  event_accumulation = cv::Mat::zeros(sensor_size_, CV_64F);
  auto it = events_.rbegin(); // Reverse iterator pointing to the last element
  const dvs_msgs::Event& lastEv = *it;
  for (int i = 0; i < N && it != events_.rend(); ++i, ++it) {
      const dvs_msgs::Event& event = *it;
      // std::cout<<event.y<<" "<<event.x<<std::endl;
      event_accumulation.at<double>(event.y,event.x) = 255;
  }
  const dvs_msgs::Event& firstEv = *it;
  // std::cout<<1000*(lastEv.ts-firstEv.ts).toSec()<<std::endl;
  // std::cout<<firstEv.ts<<"   "<<lastEv.ts<<std::endl;
  event_accumulation.convertTo(event_accumulation, CV_8U);
  cv::Mat outputImage;
  int kernelSize = 1; // Kernel size must be odd and greater than 1 (e.g., 3, 5, 7)
    cv::medianBlur(event_accumulation, outputImage, kernelSize);
  cv::Mat temp;
  cv::Mat disI = cv::Mat::zeros(sensor_size_, CV_64F);
  int ctr = 0;
  assignDistances(outputImage, disI, 10, ctr);
  // cv::GaussianBlur(outputImage, outputImage, cv::Size(5, 5), 1.4);
  // // cv::Mat row = disI.col(320).t();
  // cv::Mat row = disI.row(240);

  //   // Normalize values for display (optional, depending on your data range)
  //   cv::Mat gradient = cv::Mat::zeros(1, row.cols, CV_64F);
  //   for (int x = 0; x < row.cols - 1; ++x) {
  //       gradient.at<double>(0, x) = row.at<double>(0, x + 1) - row.at<double>(0, x);
  //   }
  //   gradient.at<double>(0, row.cols - 1) = 0; // Last point has no forward neighbor

  //   // Plot original row
  //   drawPlot(row, "Row 240 Values", "/home/yufan/Data/2025/0331/val.png", cv::Scalar(0, 0, 255));        // Red line

  //   // Plot gradient
  //   drawPlot(gradient, "Gradient of Row 240", "/home/yufan/Data/2025/0331/grad.png", cv::Scalar(255, 0, 0));
  
  // std::cout<<"Point number: "<<ctr<<std::endl;
  cv::normalize(disI, outputImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);


  // // //Save images for debug
  // std::stringstream ss;
  // ss << std::setw(10) << std::setfill('0') << external_sync_time.sec << "_" << std::setw(9) << std::setfill('0') << external_sync_time.nsec;
  // std::string filename = "/home/yufan/Data/2025/0322/exp1/DF2/" + ss.str() + ".png";
  // cv::imwrite(filename, outputImage);

  // cv::imwrite("/home/yufan/Data/2025/0331/edge.png", outputImage);

  // cv::waitKey(0);
  static cv_bridge::CvImage cv_image, cv_image3;
  cv_image.encoding = "mono8";
  cv_image.image = outputImage.clone();
  cv_image.header.stamp = external_sync_time;
  if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0)
  {
    time_surface_pub_.publish(cv_image.toImageMsg());
    cv_image3.encoding = cv_image.encoding;
    cv_image3.header.stamp = external_sync_time;
    cv_image3.image = pointSet.clone();
    pointSet_pub_.publish(cv_image3.toImageMsg());
  }
}

void TimeSurface::createEventDistanceField_ConsN(int N, const ros::Time& external_sync_time)
{
  cv::Mat event_accumulation;

  cv::Size sensor_size_2 = cv::Size(346,260);
  event_accumulation = cv::Mat::zeros(sensor_size_2, CV_64F);
  auto it = events_.begin(); //  iterator pointing to the first element
  ros::Time tsBegin; 
  tsBegin = it->ts;
  const dvs_msgs::Event& lastEv = *it;
  for (int i = 0; i < N && it != events_.end(); ++i, ++it) {
      const dvs_msgs::Event& event = *it;
      // std::cout<<event.y<<" "<<event.x<<std::endl;
      if(event.ts < external_sync_time)
        continue;
      event_accumulation.at<double>(event.y,event.x) = 255;
  }
  size_t remove_events = static_cast<size_t>(N);
  // std::cout << std::fixed << std::setprecision(9) << "is "
  //         << tsBegin.toSec() << '\n';
  // std::cout<<"all right"<<std::endl;
  // const dvs_msgs::Event& firstEv = *it;
  // std::cout<<1000*(lastEv.ts-firstEv.ts).toSec()<<std::endl;
  // std::cout<<firstEv.ts<<"   "<<lastEv.ts<<std::endl;
  event_accumulation.convertTo(event_accumulation, CV_8U);
  int suppression_window_size = 3;
  cv::Mat thinned = NMS(event_accumulation, suppression_window_size);
  cv::Mat outputImage;
  int kernelSize = 1; // Kernel size must be odd and greater than 1 (e.g., 3, 5, 7)
    cv::medianBlur(event_accumulation, outputImage, kernelSize);
  cv::Mat temp;
  cv::Mat disI = cv::Mat::zeros(sensor_size_2, CV_64F);
  int ctr = 0;
    assignDistances(outputImage, disI, 8, ctr);
  cv::normalize(disI, outputImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::bitwise_or(outputImage, event_accumulation, outputImage);
  // cv::normalize(disI, disI, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  // cv::bitwise_or(disI, outputImage, disI);
  // size_t remove_events = static_cast<size_t>(20000);
  events_.erase(events_.begin(), events_.begin() + remove_events);
  // if(oddctr){
  //   assignDistances(outputImage, disI, 12, ctr);
  //   oddctr = false;
  // }
  // else{
  //   assignDistances(outputImage, disI, 6, ctr);
  //   oddctr = true;
  // }
  
  // cv::GaussianBlur(outputImage, outputImage, cv::Size(5, 5), 1.4);
  // // cv::Mat row = disI.col(320).t();
  // cv::Mat row = disI.row(240);

  //   // Normalize values for display (optional, depending on your data range)
  //   cv::Mat gradient = cv::Mat::zeros(1, row.cols, CV_64F);
  //   for (int x = 0; x < row.cols - 1; ++x) {
  //       gradient.at<double>(0, x) = row.at<double>(0, x + 1) - row.at<double>(0, x);
  //   }
  //   gradient.at<double>(0, row.cols - 1) = 0; // Last point has no forward neighbor

  //   // Plot original row
  //   drawPlot(row, "Row 240 Values", "/home/yufan/Data/2025/0331/val.png", cv::Scalar(0, 0, 255));        // Red line

  //   // Plot gradient
  //   drawPlot(gradient, "Gradient of Row 240", "/home/yufan/Data/2025/0331/grad.png", cv::Scalar(255, 0, 0));
  
  // std::cout<<"Point number: "<<ctr<<std::endl;


  // // //Save images for debug
  // std::stringstream ss;
  // ss << std::setw(10) << std::setfill('0') << goodie << "_" << std::setw(9) << std::setfill('0') << external_sync_time.nsec;
  // std::string filename = "/home/yufan/Data/2025/1006/exp1/DF1/" + ss.str() + ".png";
  // goodie ++;
  // cv::imwrite(filename, outputImage);

  // cv::imwrite("/home/yufan/Data/2025/0331/edge.png", outputImage);

  // cv::waitKey(0);
  static cv_bridge::CvImage cv_image, cv_image3;
  cv_image.encoding = "mono8";
  cv_image.image = outputImage.clone();
  cv_image.header.stamp = tsBegin;
  if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0)
  {
    time_surface_pub_.publish(cv_image.toImageMsg());
    cv_image3.encoding = cv_image.encoding;
    cv_image3.header.stamp = tsBegin;
    cv_image3.image = pointSet.clone();
    pointSet_pub_.publish(cv_image3.toImageMsg());
  }
}
cv::Mat TimeSurface::NMS(const cv::Mat& event_accum, int kernel_size)
{
    if (kernel_size % 2 == 0) {
        kernel_size++;
    }

    cv::Mat score_map;
    cv::boxFilter(event_accum, score_map, event_accum.type(), cv::Size(kernel_size, kernel_size));

    cv::Mat dilated_map;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::dilate(score_map, dilated_map, kernel);

    cv::Mat local_maxima_mask;
    cv::compare(score_map, dilated_map, local_maxima_mask, cv::CMP_EQ);

    cv::Mat result;
    cv::bitwise_and(event_accum, local_maxima_mask, result);

    return result;
}

void TimeSurface::drawPlot(const cv::Mat& data, const std::string& title, const std::string& path, cv::Scalar lineColor = cv::Scalar(0, 0, 255)) {
  int plot_width = data.cols;
  int plot_height = 400;
  int margin = 50;

  // Find max value for scaling
  double minVal, maxVal;
  cv::minMaxLoc(data, &minVal, &maxVal);
  double y_scale = plot_height / (maxVal - minVal + 1e-9);

  // Create canvas
  cv::Mat plot(plot_height + 2 * margin, plot_width + 2 * margin, CV_8UC3, cv::Scalar(255, 255, 255));

  int base_y = margin + plot_height;

  // Draw axes
  cv::line(plot, cv::Point(margin, margin), cv::Point(margin, margin + plot_height), cv::Scalar(0, 0, 0), 1); // Y-axis
  cv::line(plot, cv::Point(margin, base_y), cv::Point(margin + plot_width, base_y), cv::Scalar(0, 0, 0), 1);   // X-axis

  // Y ticks and labels
  for (int i = 0; i <= 5; ++i) {
      double y_val = minVal + i * (maxVal - minVal) / 5.0;
      int y = cv::saturate_cast<int>(base_y - (y_val - minVal) * y_scale);
      cv::line(plot, cv::Point(margin - 5, y), cv::Point(margin + 5, y), cv::Scalar(0, 0, 0), 1);

      std::ostringstream label;
      label << std::fixed << std::setprecision(1) << y_val;
      cv::putText(plot, label.str(), cv::Point(5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
  }

  // X ticks
  for (int x = 0; x <= plot_width; x += 100) {
      int xpos = margin + x;
      cv::line(plot, cv::Point(xpos, base_y - 5), cv::Point(xpos, base_y + 5), cv::Scalar(0, 0, 0), 1);

      std::ostringstream label;
      label << x;
      cv::putText(plot, label.str(), cv::Point(xpos - 10, base_y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
  }

  // Draw line plot
  for (int x = 1; x < data.cols; ++x) {
      double val1 = data.at<double>(0, x - 1);
      double val2 = data.at<double>(0, x);

      int y1 = cv::saturate_cast<int>(base_y - (val1 - minVal) * y_scale);
      int y2 = cv::saturate_cast<int>(base_y - (val2 - minVal) * y_scale);

      cv::line(plot, cv::Point(margin + x - 1, y1), cv::Point(margin + x, y2), lineColor, 1);
  }

  // cv::imshow(title, plot);

  cv::imwrite(path, plot);
}

void TimeSurface::assignDistances(const cv::Mat& S, cv::Mat& I, int k, int &ctr) 
{
  CV_Assert(S.size() == I.size() && S.type() == CV_8U && I.type() == CV_64F);
  int radius = k / 2;
  int rows = S.rows;
  int cols = S.cols;
  int laradius = radius;
  // double laradius = sqrt(radius);
  
  // Iterate over all pixels in S
  for (int u = 0; u < rows; ++u) {
      for (int v = 0; v < cols; ++v) {
          if (S.at<uchar>(u, v) > 0) {
              ctr++;
              for (int i = -radius; i <= radius; ++i) {
                  for (int j = -radius; j <= radius; ++j) {
                      if (i*i + j*j > laradius*2)  // Circular region check
                          continue;
  
                      int ni = u + i;
                      int nj = v + j;
  
                      if (ni < 0 || ni >= rows || nj < 0 || nj >= cols)
                          continue;
  
                      double dist = static_cast<double>(sqrt(i*i+j*j));
                      I.at<double>(ni, nj) += laradius - dist;

                  }
              }
          }
      }
  }
  

}


// void TimeSurface::assignDistances(const cv::Mat& S, cv::Mat& I, int k, int &ctr)
// {
//     CV_Assert(S.size() == I.size() && S.type() == CV_8U && I.type() == CV_64F);
//     const int radius = k / 2;
//     const int r2 = radius * radius;
//     const double R = static_cast<double>(radius);  // use double for weights
//     const int rows = S.rows, cols = S.cols;

//     // (Optional) Use a smoother kernel than linear: Gaussian falloff
//     // sigma ~ radius/2 works well
//     const double sigma2 = (R*R) / 4.0;

//     for (int u = 0; u < rows; ++u) {
//         for (int v = 0; v < cols; ++v) {
//             if (S.at<uchar>(u, v) > 0) {
//                 ctr++;
//                 for (int i = -radius; i <= radius; ++i) {
//                     for (int j = -radius; j <= radius; ++j) {
//                         const int dd = i*i + j*j;
//                         if (dd > r2) continue;              // ✅ correct disk test

//                         const int ni = u + i, nj = v + j;
//                         if ((unsigned)ni >= (unsigned)rows || (unsigned)nj >= (unsigned)cols) continue;

//                         // Choose ONE of the following weights:

//                         // A) Linear ramp (softer after the radius fix)
//                         // double dist = std::sqrt((double)dd);
//                         // double w = std::max(0.0, R - dist);          // linear

//                         // B) Gaussian ramp (smoother, usually clearer)
//                         double w = std::exp(-(double)dd / (2.0 * sigma2));

//                         I.at<double>(ni, nj) += w;
//                     }
//                 }
//             }
//         }
//     }
// }


void TimeSurface::createEventAccumulation2(int N, const ros::Time& external_sync_time)
{
  cv::Mat event_accumulation;

  cv::Size sensor_size_2 = cv::Size(346,260);
  // sensor_size_2 = cv::Size(0,0);
  if (seg_pos_ >= seg_paths_.size()) {
    return;
  }
  const std::string p = static_cast<std::string>(seg_paths_[seg_pos_++]);
  event_accumulation = cv::Mat::zeros(sensor_size_2, CV_64F);
  event_accumulation = cv::imread(p, 0);
  std::cout<<p<<std::endl;
  cv::threshold(event_accumulation, event_accumulation, 0, 255, cv::THRESH_BINARY);
  // auto it = events_.rbegin(); // Reverse iterator pointing to the last element
  // const dvs_msgs::Event& lastEv = *it;
  // for (int i = 0; i < N && it != events_.rend(); ++i, ++it) {
  //     const dvs_msgs::Event& event = *it;
  //     // std::cout<<event.y<<" "<<event.x<<std::endl;
  //     event_accumulation.at<double>(event.y,event.x) = 255;
  // }
  // const dvs_msgs::Event& firstEv = *it;
  // std::cout<<1000*(lastEv.ts-firstEv.ts).toSec()<<std::endl;
  // event_accumulation.convertTo(event_accumulation, CV_8U);
  cv::Mat outputImage = event_accumulation;
  cv::normalize(outputImage, outputImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  // int kernelSize = 1; // Kernel size must be odd and greater than 1 (e.g., 3, 5, 7)
  //   cv::medianBlur(event_accumulation, outputImage, kernelSize);
  cv::Mat resizedTS;
  // cv::resize(outputImage, resizedTS, cv::Size(160, 90), 0, 0, cv::INTER_AREA);
  // cv::resize(outputImage, resizedTS, cv::Size(160, 120), 0, 0, cv::INTER_AREA);
  // cv::threshold(resizedTS, resizedTS, 1, 255, cv::THRESH_BINARY);  

  cv::Mat disI = cv::Mat::zeros(sensor_size_2, CV_64F);
  int ctr = 0;
    assignDistances(outputImage, disI, 8, ctr);
  cv::normalize(disI, outputImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);


//   int tileWidth = 34;
//   int tileHeight = 26;
// std::vector<int> nonZeroCounts;
//     for (int row = 0; row < 260; row += tileHeight) {
//         for (int col = 0; col < 346; col += tileWidth) {
//             // Define the region of interest (ROI) for each tile
//             cv::Rect tileRect(col, row, tileWidth, tileHeight);
//             cv::Mat tile = outputImage(tileRect);

//             // Count the non-zero pixels in the tile
//             int nonZeroCount = cv::countNonZero(tile);
//             nonZeroCounts.push_back(nonZeroCount);
//         }
//     }
  
    
  // cv::resize(pointSet, pointSet, cv::Size(160, 120), 0, 0, cv::INTER_AREA);
  // cv::threshold(pointSet, pointSet, 2, 255, cv::THRESH_BINARY);
  // cv::Mat binaryMask;
  // cv::threshold(outputImage, binaryMask, 5, 1, cv::THRESH_BINARY);
  // cv::Mat disField;
  // cv::distanceTransform(1-binaryMask, disField, cv::DIST_L2, 3);
  // for (int y = 0; y < disField.rows; y++) {
  //       for (int x = 0; x < disField.cols; x++) {
  //         // int tileIndex = (y / tileHeight) * (640 / tileWidth) + x / tileWidth;
  //         int disPara;

  //           disPara = 3;
  //           if (disField.at<float>(y, x) > disPara) {
  //               disField.at<float>(y, x) = 0; // Set to 0 if distance > 10
  //           }
  //           else (disField.at<float>(y, x) = disPara - disField.at<float>(y, x));
  //       }
  //   }
  // cv::normalize(disField, disField, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  //   for (int i = 0; i < 260; ++i) {
  //       for (int j = 0; j < 346; ++j) {
  //           if (outputImage.at<uchar>(i, j) == 0) { // Assuming the matrices are of type CV_8U
  //               outputImage.at<uchar>(i, j) = disField.at<uchar>(i, j);
  //               // std::cout<<"Bulabula"<<std::endl;
  //           }

  //       }
  //   }

  // // //Save images for debug
  // std::stringstream ss;
  // ss << std::setw(10) << std::setfill('0') << external_sync_time.sec << "_" << std::setw(9) << std::setfill('0') << external_sync_time.nsec;
  // std::string filename = "/home/yufan/Data/2025/0212/exp1/DF2/" + ss.str() + ".png";
  // cv::imwrite(filename, outputImage);

  // cv::imwrite("/home/yufan/Data/2025/0228/ps.png", outputImage);
  // cv::waitKey(0);

  size_t remove_events = static_cast<size_t>(20000);
  events_.erase(events_.begin(), events_.begin() + remove_events);
  auto it = events_.begin(); //  iterator pointing to the first element
  ros::Time tsBegin; 
  tsBegin = it->ts;
  static cv_bridge::CvImage cv_image, cv_image3;
  cv_image.encoding = "mono8";
  cv_image.image = outputImage.clone();
  cv_image.header.stamp = tsBegin;
  if (time_surface_mode_ == BACKWARD && bCamInfoAvailable_ && time_surface_pub_.getNumSubscribers() > 0)
  {
    time_surface_pub_.publish(cv_image.toImageMsg());
    cv_image3.encoding = cv_image.encoding;
    cv_image3.header.stamp = tsBegin;
    cv_image3.image = pointSet.clone();
    pointSet_pub_.publish(cv_image3.toImageMsg());
  }
}

void TimeSurface::syncCallback(const std_msgs::TimeConstPtr& msg)
{
  // std::cout<<"Bulala"<<std::endl;
  if(oddctr) return;
  // std::cout<<"Bulala2"<<std::endl;
  if(events_.size() < 10*eventNumber_)
    return;
#ifdef ESVO_TS_LOG
    TicToc tt;
    tt.tic();
#endif
  if((events_.back().ts - sync_time_).toSec() < 0)
    return;

    if(NUM_THREAD_TS == 1)
      createTimeSurfaceAtTime(sync_time_);
    if(NUM_THREAD_TS > 1)
      // createTimeSurfaceAtTime_hyperthread(sync_time_);
      createEventDistanceField_ConsN(eventNumber_, sync_time_);
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

  std::cout<<"K parameters are "<<msg->K[0]<<" "<<msg->K[2]<<std::endl;
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

#ifdef ESVO_TS_LOG
    TicToc tt;
    tt.tic();
#endif
  if(!bSensorInitialized_)
    init(msg->width, msg->height);
  oddctr = false;
  // // std::cout<<"Last event timestamp is "<<std::endl;
  // for(const dvs_msgs::Event& e : msg->events)
  // {
  //   events_.push_back(e);
  //   const dvs_msgs::Event& last_event = events_.back();
  //   pEventTs_[last_event.x + last_event.y*msg->width] = last_event.ts.toSec();
  // }

  // clearEventQueue();

}

void TimeSurface::clearEventQueue()
{
  static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 500000000;
  if (events_.size() > MAX_EVENT_QUEUE_LENGTH)
  {
    size_t remove_events = events_.size() - MAX_EVENT_QUEUE_LENGTH;
    events_.erase(events_.begin(), events_.begin() + remove_events);
    std::cout<<"Cut off!"<<std::endl;
  }
}

} // namespace esvo_time_surface