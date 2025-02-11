#include <esvo_core/esvo_Tracking.h>
#include <esvo_core/tools/TicToc.h>
#include <esvo_core/tools/params_helper.h>
#include <minkindr_conversions/kindr_tf.h>
#include <tf/transform_broadcaster.h>
#include <sys/stat.h>

// #define ESVO_CORE_TRACKING_DEBUG
// #define ESVO_CORE_TRACKING_LOG

namespace esvo_core
{
esvo_Tracking::esvo_Tracking(
  const ros::NodeHandle &nh,
  const ros::NodeHandle &nh_private):
  nh_(nh),
  pnh_(nh_private),
  it_(nh),
  TS_left_sub_(nh_, "time_surface_left", 10),//Because Bobbin dataset only has one camera
  TS_right_sub_(nh_, "time_surface_left", 10),
  // Point_set_sub_(nh_, "point_set", 10),
  TS_sync_(ExactSyncPolicy(10), TS_left_sub_, TS_left_sub_),
  calibInfoDir_(tools::param(pnh_, "calibInfoDir", std::string(""))),
  camSysPtr_(new CameraSystem(calibInfoDir_, false)),
  rpConfigPtr_(new RegProblemConfig(
    tools::param(pnh_, "patch_size_X", 25),
    tools::param(pnh_, "patch_size_Y", 25),
    tools::param(pnh_, "kernelSize", 15),
    tools::param(pnh_, "LSnorm", std::string("l2")),
    tools::param(pnh_, "huber_threshold", 10.0),
    tools::param(pnh_, "invDepth_min_range", 0.0),
    tools::param(pnh_, "invDepth_max_range", 0.0),
    tools::param(pnh_, "MIN_NUM_EVENTS", 1000),
    tools::param(pnh_, "MAX_REGISTRATION_POINTS", 500),
    tools::param(pnh_, "BATCH_SIZE", 200),
    tools::param(pnh_, "MAX_ITERATION", 10))),
  rpType_((RegProblemType)((size_t)tools::param(pnh_, "RegProblemType", 0))),
  rpSolver_(camSysPtr_, rpConfigPtr_, rpType_, NUM_THREAD_TRACKING),
  ESVO_System_Status_("INITIALIZATION"),
  ets_(IDLE),
  pc_(new PointCloud())
{
  // offline data
  dvs_frame_id_        = tools::param(pnh_, "dvs_frame_id", std::string("dvs"));
  world_frame_id_      = tools::param(pnh_, "world_frame_id", std::string("world"));
  pc_->header.frame_id = world_frame_id_;

  /**** online parameters ***/
  tracking_rate_hz_    = tools::param(pnh_, "tracking_rate_hz", 100);
  TS_HISTORY_LENGTH_  = tools::param(pnh_, "TS_HISTORY_LENGTH", 100);
  REF_HISTORY_LENGTH_  = tools::param(pnh_, "REF_HISTORY_LENGTH", 5);
  bSaveTrajectory_     = tools::param(pnh_, "SAVE_TRAJECTORY", false);
  bVisualizeTrajectory_ = tools::param(pnh_, "VISUALIZE_TRAJECTORY", true);
  resultPath_             = tools::param(pnh_, "PATH_TO_SAVE_TRAJECTORY", std::string());
  evsModelPath_             = tools::param(pnh_, "PATH_TO_LOAD_3DModel", std::string());
  camIntrinsicPath_            = tools::param(pnh_, "PATH_TO_CAMERA_INTRINSICS", std::string());
  nh_.setParam("/ESVO_SYSTEM_STATUS", ESVO_System_Status_);
  
  Point_set_sub_ = nh_.subscribe("point_set", 10, &esvo_Tracking::pointsetCallback, this);
  // online data callbacks
  events_left_sub_  = nh_.subscribe<dvs_msgs::EventArray>(
    "events_left", 0, &esvo_Tracking::eventsCallback, this);
  TS_sync_.registerCallback(boost::bind(&esvo_Tracking::timeSurfaceCallback, this, _1, _2));
  tf_ = std::make_shared<tf::Transformer>(true, ros::Duration(100.0));
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/esvo_tracking/pose_pub", 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/esvo_tracking/trajectory", 1);
  map_sub_ = nh_.subscribe("pointcloud", 0, &esvo_Tracking::refMapCallback, this);// local map in the ref view.
  stampedPose_sub_ = nh_.subscribe("stamped_pose", 0, &esvo_Tracking::stampedPoseCallback, this);// for accessing the pose of the ref view.
  pointSet_pub_ = it_.advertise("/Depth_Pointset", 1);


  //orfcv
  cam_omni = CameraOmni(YAML::LoadFile(camIntrinsicPath_));
  // std::cout<<"The camera intrinsic model is "<<cam_omni.xi<<" "<<cam_omni.px<< " "<< cam_omni.k[4]<<std::endl;
  cam = new COmni(cam_omni.px, cam_omni.py, cam_omni.u0, cam_omni.v0, cam_omni.xi, cam_omni.k[0], cam_omni.k[1], cam_omni.k[2], cam_omni.k[3]);
  cam->setActiveDistorsionParameters(true, true, false, false, false);
  // std::cout<<cam_omni.px<<" "<<cam_omni.py<<" "<<cam_omni.u0<<" "<<cam_omni.v0<<" "<<cam_omni.xi<<std::endl;
  moteur = new gcOgre(cam, cam_omni.width, cam_omni.height, "/home/yufan/Related/Dependency/ogre-1.12.2/OgreConfigs/");
  moteur->init(); 
  moteur->loadPointCloud("objecttotrack", evsModelPath_);
  moteur->setClipDistances(cam_omni.clip_near, cam_omni.clip_far);
    // std::cout<<"Pose = "<<cam_omni.pose[0]<<" "<<cam_omni.pose[6]<<std::endl;
  cMo = toHomogeneousMatrix(cam_omni.pose).inverse();

    // vpPoseVector pv(cMo);
    // std::cout << "cMo = " << pv.t() << std::endl;
  if (moteur->continueRendering())
  {
      ROS_INFO("tentative rendu");
      moteur->display(&cMo);
  }
  else
  {
      ROS_INFO("probleme rendu");
      exit(12);
  }
  kf_omni.kfresize(cam_omni.width, cam_omni.height);
  moteur->getInternalImage(kf_omni.I);
  moteur->getInternalImageZ(kf_omni.Idepth);

  // std::cout<<"The width you want is "<<camSysPtr_->cam_left_ptr_->width_<<std::endl;
  vpImageConvert::convert(kf_omni.I, kf_I, true);
  // vpImageConvert::convert(kf_omni.Idepth, kf_depth);
  // image_gradient(kf_I, kf_grad);
  // int gradCounter = 0;
  kf_depth = cv::Mat::zeros(cam_omni.height, cam_omni.width, CV_64FC1);
  pc_->clear();
  pc_->reserve(5000);
  for (int i = 0; i < cam_omni.height; i++)
  {
      for (int j = 0; j < cam_omni.width; j++)
      {
          // if ((kf.gradient.at<Vector2d>(i, j)[0] != 0 || kf.gradient.at<Vector2d>(i, j)[1] != 0) && (kf.depth.at<double>(i, j) > 0))
          // if (kf_grad.at<Vector2d>(i, j)[0] != 0 || kf_grad.at<Vector2d>(i, j)[1] != 0)
          // {
          //     gradCounter++;
          //     // std::cout << "i, j = " << i << " " << j << std::endl;
          //     // cv::Vec3 test = kf.gradient.at<cv::Vec3>(i, j);
          //     // std::cout << "Grad_x = " << kf.gradient.at<Vector2d>(i, j)[0] << "  Grad_y = " << kf.gradient.at<Vector2d>(i, j)[1] << std::endl;
          // }
          kf_depth.at<double>(i, j) = double(kf_omni.Idepth[i][j]);
      }
  }

    // cv::imwrite("/home/yufan/Data/2024/0506/kf.png", kf_I);
  cv::minMaxLoc(kf_depth, &mMin, &mMax, &minP, &maxP);
  mMin = 0;
  psFlag = true;
  // std::cout<<kf_depth<<std::endl;
  // std::cout<<"The pixels with gradient is "<<gradCounter<<std::endl;
 
  // visualize the point set and depth map
  // pointSet = cv::imread("/home/yufan/Data/experiments/ESVO/EVS/edgemap.png", 0);
  // // std::cout<<pointSet<<std::endl;
  // static cv_bridge::CvImage cv_image;
  // cv_image.encoding = "mono8";
  // cv_image.image = pointSet.clone();

  // // ros::Time sync_time_ = ros::Time((long int)startTimeSec_, (long int)startTimeNsec_);
  // cv_image.header.stamp = ros::Time(1713838578, 314152240);
  // cv_image.header.stamp = ros::Time::now();

  // pointSet_pub_.publish(cv_image.toImageMsg());


 
  /*** For Visualization and Test ***/
  reprojMap_pub_left_  = it_.advertise("Reproj_Map_Left", 1);
  rpSolver_.setRegPublisher(&reprojMap_pub_left_);
  evs_pub_  = nh_.advertise<geometry_msgs::Twist>("/esvo_tracking/evs", 1);
  rpSolver_.setevsPublisher(evs_pub_);

/////////////////////////////////////////////////
  //Yufan add this 
  // std_msgs::Header header;
  // header.stamp = ros::Time::now();
  // sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(header, "bgr8", kf_grad).toImageMsg();
  // reprojMap_pub_left_.publish(msg2);



  /*** Tracker ***/
  T_world_cur_ = Eigen::Matrix<double,4,4>::Identity();
  std::thread TrackingThread(&esvo_Tracking::TrackingLoop, this);
  TrackingThread.detach();
}

esvo_Tracking::~esvo_Tracking()
{
  pose_pub_.shutdown();
  pointSet_pub_.shutdown();
  evs_pub_.shutdown();
}

void esvo_Tracking::TrackingLoop()
{
  ros::Rate r(tracking_rate_hz_);
  while(ros::ok())
  {
    // Keep Idling
    // std::cout << "amebabababa..."<<std::endl;
    if(refPCMap_.size() < 1 || TS_history_.size() < 1)
    {
      // std::cout << "Sleeping..." << std::endl;
      // r.sleep();
      continue;
    }
    // Reset
    nh_.getParam("/ESVO_SYSTEM_STATUS", ESVO_System_Status_);
    // if(ESVO_System_Status_ == "INITIALIZATION" && ets_ == WORKING)// This is true when the system is reset from dynamic reconfigure
    // {
    //   reset();
    //   // r.sleep();
    //   continue;
    // }
    if(ESVO_System_Status_ == "TERMINATE")
    {
      LOG(INFO) << "The tracking node is terminated manually...";
      break;
    }

      // std::cout<<refPCMap_.size()<<" "<<TS_history_.size()<<std::endl;
    // Data Transfer (If mapping node had published refPC.)
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if(ref_.t_.toSec() < refPCMap_.rbegin()->first.toSec())// new reference map arrived
      {
        refDataTransferring();
        // std::cout<<"Times are "<< ref_.t_.toSec() <<" "<< refPCMap_.rbegin()->first.toSec()<<std::endl;
      }

      // std::cout<<cur_.t_.toSec() - TS_history_.rbegin()->first.toSec()<<std::endl;
      if(cur_.t_.toSec() < TS_history_.rbegin()->first.toSec())// new observation arrived
      {
        // if(ref_.t_.toSec() >= TS_history_.rbegin()->first.toSec())
        if(ref_.t_.toSec() > TS_history_.rbegin()->first.toSec())
        {
          LOG(INFO) << "The time_surface observation should be obtained after the reference frame";
          exit(-1);
        }
        if(!curDataTransferring())
          continue;
      }
      else
        continue;
    }
    // create new regProblem

    TicToc tt;
    double t_resetRegProblem, t_solve, t_pub_result, t_pub_gt;
#ifdef  ESVO_CORE_TRACKING_DEBUG
    tt.tic();
#endif
    if(rpSolver_.resetRegProblem(&ref_, &cur_))
    {
#ifdef  ESVO_CORE_TRACKING_DEBUG
      t_resetRegProblem = tt.toc();
      tt.tic();
#endif
      if(ets_ == IDLE)
        ets_ = WORKING;
      if(ESVO_System_Status_ != "WORKING")
        nh_.setParam("/ESVO_SYSTEM_STATUS", "WORKING");
      if(rpType_ == REG_NUMERICAL)
        rpSolver_.solve_numerical();
      if(rpType_ == REG_ANALYTICAL)
        rpSolver_.solve_analytical();
#ifdef ESVO_CORE_TRACKING_DEBUG
      t_solve = tt.toc();
      tt.tic();
#endif
      T_world_cur_ = cur_.tr_.getTransformationMatrix();
      publishPose(cur_.t_, cur_.tr_);
      if(bVisualizeTrajectory_)
        publishPath(cur_.t_, cur_.tr_);
#ifdef ESVO_CORE_TRACKING_DEBUG
      t_pub_result = tt.toc();
#endif

      // save result and gt if available.
      if(bSaveTrajectory_)
      {
        // save results to listPose and listPoseGt
        lTimestamp_.push_back(std::to_string(cur_.t_.toSec()));
        lPose_.push_back(cur_.tr_.getTransformationMatrix());
      }
    }
    else
    {
      nh_.setParam("/ESVO_SYSTEM_STATUS", "INITIALIZATION");
      ets_ = IDLE;
//      LOG(INFO) << "Tracking thread is IDLE";
    }

#ifdef  ESVO_CORE_TRACKING_LOG
    double t_overall_count = 0;
    t_overall_count = t_resetRegProblem + t_solve + t_pub_result;
    LOG(INFO) << "\n";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "--------------------Tracking Computation Cost---------------";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "ResetRegProblem: " << t_resetRegProblem << " ms, (" << t_resetRegProblem / t_overall_count * 100 << "%).";
    LOG(INFO) << "Registration: " << t_solve << " ms, (" << t_solve / t_overall_count * 100 << "%).";
    LOG(INFO) << "pub result: " << t_pub_result << " ms, (" << t_pub_result / t_overall_count * 100 << "%).";
    LOG(INFO) << "Total Computation (" << rpSolver_.lmStatics_.nPoints_ << "): " << t_overall_count << " ms.";
    LOG(INFO) << "------------------------------------------------------------";
    LOG(INFO) << "------------------------------------------------------------";
#endif
  // cv::waitKey(0);
    // r.sleep();
  }// while

  if(bSaveTrajectory_)
  {
    struct stat st;
    if( stat(resultPath_.c_str(), &st) == -1 )// there is no such dir, create one
    {
      LOG(INFO) << "There is no such directory: " << resultPath_;
      _mkdir(resultPath_.c_str());
      LOG(INFO) << "The directory has been created!!!";
    }
    LOG(INFO) << "pose size: " << lPose_.size();
    LOG(INFO) << "refPCMap_.size(): " << refPCMap_.size() << ", TS_history_.size(): " << TS_history_.size();
    saveTrajectory(resultPath_ + "result.txt");
  }
}

//Transfer the depth map data to ref_ frame. The data source is refPCMAP_. tr is set to identity matrix
bool
esvo_Tracking::refDataTransferring()
{
  // load reference info
  ref_.t_ = refPCMap_.rbegin()->first;

  nh_.getParam("/ESVO_SYSTEM_STATUS", ESVO_System_Status_);
//  LOG(INFO) << "SYSTEM STATUS(T"
  if(ESVO_System_Status_ == "INITIALIZATION" && ets_ == IDLE)
    ref_.tr_.setIdentity();
  if(ESVO_System_Status_ == "WORKING" || (ESVO_System_Status_ == "INITIALIZATION" && ets_ == WORKING))
  {
    if(!getPoseAt(ref_.t_, ref_.tr_, dvs_frame_id_))
    {
      LOG(INFO) << "ESVO_System_Status_: " << ESVO_System_Status_ << ", ref_.t_: " << ref_.t_.toNSec();
      LOG(INFO) << "Logic error ! There must be a pose for the given timestamp, because mapping has been finished.";
      exit(-1);
      return false;
    }
  }

  size_t numPoint = refPCMap_.rbegin()->second->size();
  ref_.vPointXYZPtr_.clear();
  ref_.vPointXYZPtr_.reserve(numPoint);
  auto PointXYZ_begin_it = refPCMap_.rbegin()->second->begin();
  auto PointXYZ_end_it   = refPCMap_.rbegin()->second->end();
  while(PointXYZ_begin_it != PointXYZ_end_it)
  {
    ref_.vPointXYZPtr_.push_back(PointXYZ_begin_it.base());// Copy the pointer of the pointXYZ
    PointXYZ_begin_it++;
  }
  // std::cout<<ref_.tr_<<std::endl;
  return true;
}

bool
esvo_Tracking::curDataTransferring()
{
  // load current observation
  auto ev_last_it = EventBuffer_lower_bound(events_left_, cur_.t_);
  auto TS_it = TS_history_.rbegin();

  // TS_history may not be updated before the tracking loop excutes the data transfering
  if(cur_.t_ == TS_it->first)
    return false;
  cur_.t_ = TS_it->first;
  cur_.pTsObs_ = &TS_it->second;

  // std::cout<<"sync time = "<<cur_.t_<<std::endl;
  // std::cout << "Current time: " << cur_.t_ .sec << " seconds and " 
  //             << cur_.t_ .nsec << " nanoseconds" << std::endl;
  nh_.getParam("/ESVO_SYSTEM_STATUS", ESVO_System_Status_);
  if(ESVO_System_Status_ == "INITIALIZATION" && ets_ == IDLE)
  {
    cur_.tr_ = ref_.tr_;
//    LOG(INFO) << "(IDLE) Assign cur's ("<< cur_.t_.toNSec() << ") pose with ref's at " << ref_.t_.toNSec();
    // LOG(INFO) << " " << cur_.tr_.getTransformationMatrix() << " ";
  }
  if(ESVO_System_Status_ == "WORKING" || (ESVO_System_Status_ == "INITIALIZATION" && ets_ == WORKING))
  {
    cur_.tr_ = Transformation(T_world_cur_);
//    LOG(INFO) << "(WORKING) Assign cur's ("<< cur_.t_.toNSec() << ") pose with T_world_cur.";
  }
  // Count the number of events occuring since the last observation.
  auto ev_cur_it = EventBuffer_lower_bound(events_left_, cur_.t_);
  cur_.numEventsSinceLastObs_ = std::distance(ev_last_it, ev_cur_it) + 1;
  return true;
}

void esvo_Tracking::reset()
{
  // clear all maintained data
  ets_ = IDLE;
  TS_id_ = 0;
  TS_history_.clear();
  refPCMap_.clear();
  events_left_.clear();
}


/********************** Callback functions *****************************/
void esvo_Tracking::refMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // cam_omni = CameraOmni(YAML::LoadFile(camIntrinsicPath_));
  // std::cout<<"The camera intrinsic model is "<<cam_omni.xi<<" "<<cam_omni.px<< " "<< cam_omni.k[4]<<std::endl;
  // cam = new COmni(cam_omni.px, cam_omni.py, cam_omni.u0, cam_omni.v0, cam_omni.xi, cam_omni.k[0], cam_omni.k[1], cam_omni.k[2], cam_omni.k[3], cam_omni.k[4]);
  // moteur = new gcOgre(cam, cam_omni.width, cam_omni.height);
  // moteur->init(); 
  // std::cout<<"The width and height are"<<cam_omni.width<<" "<<cam_omni.height<<std::endl;
  // moteur->loadPointCloud("objecttotrack", bobbinModelPath_);
  std::lock_guard<std::mutex> lock(data_mutex_);
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*msg, pcl_pc);
  PointCloud::Ptr PC_ptr(new PointCloud());
  pcl::fromPCLPointCloud2(pcl_pc, *PC_ptr);
  refPCMap_.emplace(msg->header.stamp, PC_ptr);
  while(refPCMap_.size() > REF_HISTORY_LENGTH_)
  {
    auto it = refPCMap_.begin();
    refPCMap_.erase(it);
  }
}

void esvo_Tracking::eventsCallback(
  const dvs_msgs::EventArray::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // add new ones and remove old ones
  for(const dvs_msgs::Event& e : msg->events)
  {
    events_left_.push_back(e);
    int i = events_left_.size() - 2;
    while(i >= 0 && events_left_[i].ts > e.ts) // we may have to sort the queue, just in case the raw event messages do not come in a chronological order.
    {
      events_left_[i+1] = events_left_[i];
      i--;
    }
    events_left_[i+1] = e;
  }
  clearEventQueue();
}

void esvo_Tracking::clearEventQueue()
{
  static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 5000000;
  if (events_left_.size() > MAX_EVENT_QUEUE_LENGTH)
  {
    size_t remove_events = events_left_.size() - MAX_EVENT_QUEUE_LENGTH;
    events_left_.erase(events_left_.begin(), events_left_.begin() + remove_events);
  }
}

void
esvo_Tracking::timeSurfaceCallback(
  const sensor_msgs::ImageConstPtr &time_surface_left,
  const sensor_msgs::ImageConstPtr &time_surface_right)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
  try
  {
    cv_ptr_left  = cv_bridge::toCvCopy(time_surface_left,  sensor_msgs::image_encodings::MONO8);
    cv_ptr_right = cv_bridge::toCvCopy(time_surface_right, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // push back the most current TS.
  ros::Time t_new_ts = time_surface_left->header.stamp;
  TS_history_.emplace(t_new_ts, TimeSurfaceObservation(cv_ptr_left, cv_ptr_right, TS_id_, false));
  TS_id_++;

  // keep TS_history_'s size constant
  while(TS_history_.size() > TS_HISTORY_LENGTH_)
  {
    auto it = TS_history_.begin();
    TS_history_.erase(it);
  }
  // std::cout<<TS_history_.size()<<std::endl;
}

void 
esvo_Tracking::pointsetCallback(const sensor_msgs::ImageConstPtr &point_set)
{
  if(psFlag == false)
    return;
  cv::Mat img;
  cv_bridge::CvImagePtr cv_ptr_ps;
  cv_ptr_ps = cv_bridge::toCvCopy(point_set, sensor_msgs::image_encodings::MONO8);
  cv_ptr_ps->image.copyTo(img);
  // std::cout<<img.type()<<std::endl;
  cv::cvtColor(img, img, CV_GRAY2BGR);
  // std::cout<<img<<std::endl;
  // int ctr = 0;
  for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            // if ((abs(kf.gradient.at<Vector2d>(i, j)[0] * kf.gradient.at<Vector2d>(i, j)[1]) > 10) && (kf.depth.at<double>(i, j) > 0))
            if (img.at<cv::Vec3b>(i,j)[1] == 255 )
            {
                double z = kf_depth.at<double>(i,j);
                // std::cout<<z<<" ";
                visualizor_.DrawPoint(1.0 / z, 1.0 / 0.3, 1.0 / mMax,  Eigen::Vector2d(j,i), img);
                // ctr ++;
                Eigen::Vector3d p_world;
                Eigen::Vector2d p_cam(j, i);
                camSysPtr_->cam_left_ptr_->cam2World(p_cam, 1.0 / z, p_world);
                // Eigen::Vector2d p_tmp;
                // camSysPtr_->cam_left_ptr_->world2Cam(p_world, p_tmp);
                // std::cout<<p_tmp<<std::endl;
                pc_->push_back(pcl::PointXYZ(p_world(0), p_world(1), p_world(2)));
            }
        }
    }
  
  refPCMap_.emplace(cv_ptr_ps->header.stamp, pc_);
  // std::cout<<refPCMap_.size()<<std::endl;
  // std::cout<<"The point set number is "<<ctr<<std::endl;
  std_msgs::Header header;
  header.stamp = cv_ptr_ps->header.stamp;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
  // static cv_bridge::CvImage cv_image;
  // cv_image.encoding = "mono8";
  // cv_image.image = kf_I.clone();
  // cv_image.header.stamp = cv_ptr_ps->header.stamp;
  // pointSet_pub_.publish(cv_image.toImageMsg());
  pointSet_pub_.publish(msg);
  psFlag = false;
  // cv::waitKey(0);
};

void esvo_Tracking::stampedPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // add pose to tf
  tf::Transform tf(
    tf::Quaternion(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w),
    tf::Vector3(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z));
  tf::StampedTransform st(tf, msg->header.stamp, msg->header.frame_id, dvs_frame_id_.c_str());
  tf_->setTransform(st);
  // broadcast the tf such that the nav_path messages can find the valid fixed frame "map".
  static tf::TransformBroadcaster br;
  br.sendTransform(st);
}

bool
esvo_Tracking::getPoseAt(
  const ros::Time &t, esvo_core::Transformation &Tr, const std::string &source_frame)
{
  std::string* err_msg = new std::string();
  if(!tf_->canTransform(world_frame_id_, source_frame, t, err_msg))
  {
    LOG(WARNING) << t.toNSec() << " : " << *err_msg;
    delete err_msg;
    return false;
  }
  else
  {
    tf::StampedTransform st;
    tf_->lookupTransform(world_frame_id_, source_frame, t, st);
    tf::transformTFToKindr(st, &Tr);
    return true;
  }
}

/************ publish results *******************/
void esvo_Tracking::publishPose(const ros::Time &t, Transformation &tr)
{
  geometry_msgs::PoseStampedPtr ps_ptr(new geometry_msgs::PoseStamped());
  ps_ptr->header.stamp = t;
  ps_ptr->header.frame_id = world_frame_id_;
  ps_ptr->pose.position.x = tr.getPosition()(0);
  ps_ptr->pose.position.y = tr.getPosition()(1);
  ps_ptr->pose.position.z = tr.getPosition()(2);
  ps_ptr->pose.orientation.x = tr.getRotation().x();
  ps_ptr->pose.orientation.y = tr.getRotation().y();
  ps_ptr->pose.orientation.z = tr.getRotation().z();
  ps_ptr->pose.orientation.w = tr.getRotation().w();
  pose_pub_.publish(ps_ptr);
}

void esvo_Tracking::publishPath(const ros::Time& t, Transformation& tr)
{
  geometry_msgs::PoseStampedPtr ps_ptr(new geometry_msgs::PoseStamped());
  ps_ptr->header.stamp = t;
  ps_ptr->header.frame_id = world_frame_id_;
  ps_ptr->pose.position.x = tr.getPosition()(0);
  ps_ptr->pose.position.y = tr.getPosition()(1);
  ps_ptr->pose.position.z = tr.getPosition()(2);
  ps_ptr->pose.orientation.x = tr.getRotation().x();
  ps_ptr->pose.orientation.y = tr.getRotation().y();
  ps_ptr->pose.orientation.z = tr.getRotation().z();
  ps_ptr->pose.orientation.w = tr.getRotation().w();

  path_.header.stamp = t;
  path_.header.frame_id = world_frame_id_;
  path_.poses.push_back(*ps_ptr);
  path_pub_.publish(path_);
}

void
esvo_Tracking::saveTrajectory(const std::string &resultDir)
{
  LOG(INFO) << "Saving trajectory to " << resultDir << " ......";

  std::ofstream  f;
  f.open(resultDir.c_str(), std::ofstream::out);
  if(!f.is_open())
  {
    LOG(INFO) << "File at " << resultDir << " is not opened, save trajectory failed.";
    exit(-1);
  }
  f << std::fixed;

  std::list<Eigen::Matrix<double,4,4>,
    Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > >::iterator result_it_begin = lPose_.begin();
  std::list<Eigen::Matrix<double,4,4>,
    Eigen::aligned_allocator<Eigen::Matrix<double,4,4> > >::iterator result_it_end = lPose_.end();
  std::list<std::string>::iterator  ts_it_begin = lTimestamp_.begin();

  for(;result_it_begin != result_it_end; result_it_begin++, ts_it_begin++)
  {
    Eigen::Matrix3d Rwc_result;
    Eigen::Vector3d twc_result;
    Rwc_result = (*result_it_begin).block<3,3>(0,0);
    twc_result = (*result_it_begin).block<3,1>(0,3);
    Eigen::Quaterniond q(Rwc_result);
    f << *ts_it_begin << " " << std::setprecision(9) << twc_result.transpose() << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  f.close();
  LOG(INFO) << "Saving trajectory to " << resultDir << ". Done !!!!!!.";
}

vpHomogeneousMatrix esvo_Tracking::toHomogeneousMatrix(double *s)
{
    vpHomogeneousMatrix mat;
    vpTranslationVector vec(s[0], s[1], s[2]);
    vpRotationMatrix rmat;

    // double a = s.pose.orientation.x();
    // double b = s.pose.orientation.y();
    // double c = s.pose.orientation.z();
    // double d = s.pose.orientation.w();
    double a = s[6];
    double b = s[3];
    double c = s[4];
    double d = s[5];
    rmat[0][0] = a * a + b * b - c * c - d * d;
    rmat[0][1] = 2 * b * c - 2 * a * d;
    rmat[0][2] = 2 * a * c + 2 * b * d;

    rmat[1][0] = 2 * a * d + 2 * b * c;
    rmat[1][1] = a * a - b * b + c * c - d * d;
    rmat[1][2] = 2 * c * d - 2 * a * b;

    rmat[2][0] = 2 * b * d - 2 * a * c;
    rmat[2][1] = 2 * a * b + 2 * c * d;
    rmat[2][2] = a * a - b * b - c * c + d * d;

    mat.buildFrom(vec, rmat);

    return mat;
}
}// namespace esvo_core
