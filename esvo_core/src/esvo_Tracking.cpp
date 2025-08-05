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
  pc_(new PointCloud()),
  fake_time_(ros::Time(1.0))
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
  
  // Point_set_sub_ = nh_.subscribe("point_set", 10, &esvo_Tracking::pointsetCallback, this);
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
  cam->setActiveDistorsionParameters(true, false, false, false, false);
  // std::cout<<cam_omni.px<<" "<<cam_omni.py<<" "<<cam_omni.u0<<" "<<cam_omni.v0<<" "<<cam_omni.xi<<std::endl;
  // moteur = new gcOgre(cam, cam_omni.width, cam_omni.height, "/home/yufan/Dependency/ogre-1.12.2/OgreConfigs/");
  // moteur->init(); 
  // moteur->loadPointCloud("objecttotrack", evsModelPath_);
  // moteur->setClipDistances(cam_omni.clip_near, cam_omni.clip_far);
  //   // std::cout<<"Pose = "<<cam_omni.pose[0]<<" "<<cam_omni.pose[6]<<std::endl;
  cMo = toHomogeneousMatrix(cam_omni.pose).inverse();

  //   // vpPoseVector pv(cMo);
  //   // std::cout << "cMo = " << pv.t() << std::endl;

  // if (moteur->continueRendering())
  // {
  //     ROS_INFO("tentative rendu");
  //     moteur->display(&cMo);
  // }
  // else
  // {
  //     ROS_INFO("probleme rendu");
  //     exit(12);
  // }
  
  // kf_omni.kfresize(cam_omni.width, cam_omni.height);
  // moteur->getInternalImage(kf_omni.I);
  // moteur->getInternalImageZ(kf_omni.Idepth);

  // // std::cout<<"The width you want is "<<camSysPtr_->cam_left_ptr_->width_<<std::endl;
  // vpImageConvert::convert(kf_omni.I, kf_I, true);
  // // vpImageConvert::convert(kf_omni.Idepth, kf_depth);
  // // image_gradient(kf_I, kf_grad);
  // // int gradCounter = 0;
  // kf_depth = cv::Mat::zeros(cam_omni.height, cam_omni.width, CV_64FC1);
  // pc_->clear();
  // pc_->reserve(5000);
  // for (int i = 0; i < cam_omni.height; i++)
  // {
  //     for (int j = 0; j < cam_omni.width; j++)
  //     {
  //         // if ((kf.gradient.at<Vector2d>(i, j)[0] != 0 || kf.gradient.at<Vector2d>(i, j)[1] != 0) && (kf.depth.at<double>(i, j) > 0))
  //         // if (kf_grad.at<Vector2d>(i, j)[0] != 0 || kf_grad.at<Vector2d>(i, j)[1] != 0)
  //         // {
  //         //     gradCounter++;
  //         //     // std::cout << "i, j = " << i << " " << j << std::endl;
  //         //     // cv::Vec3 test = kf.gradient.at<cv::Vec3>(i, j);
  //         //     // std::cout << "Grad_x = " << kf.gradient.at<Vector2d>(i, j)[0] << "  Grad_y = " << kf.gradient.at<Vector2d>(i, j)[1] << std::endl;
  //         // }
  //         kf_depth.at<double>(i, j) = double(kf_omni.Idepth[i][j]);
  //     }
  // }
  // cv::GaussianBlur(kf_I, blurred, cv::Size(7, 7), 2.0);
  // cv::Canny(kf_I, edges, 150, 300);
 
  //   cv::imwrite("/home/yufan/Data/2025/0606/edges1.png", edges);
  // cv::minMaxLoc(kf_depth, &mMin, &mMax, &minP, &maxP);
  // mMin = 0; 
  psFlag = true; 
  // refreshDepth(edges,kf_depth);
  // std::cout<<mMin<<" "<<mMax<<std::endl;
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

// Define small pose shift: 1cm along X, 1° around Z
c0Mo = cMo;  // Save original pose
double dx = 0.01; // 1 cm shift per frame along X
double dy = 0.0;
double dz = 0.0;

double angle_x = 0.0;
double angle_y = 0.0;
double angle_z = 0.0;

vpRotationMatrix R;
R.buildFrom(angle_x, angle_y, angle_z); // Identity rotation

vpTranslationVector T(dx, dy, dz); // X shift only

//delta_i;
delta_i.buildFrom(T, R);
// for (int i = 0; i < 100; ++i) rerender works in header
// {
//   rerender();
// }

// Loop over 100 unique perturbations
// for (int i = 0; i < 100; ++i)
// {
//     // Create unique small translation and rotation per image
//     double dx = 0.01 * std::sin(i * 0.1);  // e.g., ~[-1cm, +1cm]
//     double dy = 0.01 * std::cos(i * 0.1);
//     double dz = 0.005 * std::sin(i * 0.05);

//     double angle_x = 0.5 * M_PI / 180.0 * std::sin(i * 0.07); // ≤ 0.5 deg
//     double angle_y = 0.5 * M_PI / 180.0 * std::cos(i * 0.04);
//     double angle_z = 0.5 * M_PI / 180.0 * std::sin(i * 0.09);

//     // Build rotation and translation
//     vpRotationMatrix R;
//     R.buildFrom(angle_x, angle_y, angle_z);
//     vpTranslationVector T(dx, dy, dz);

//     // delta_i.buildFrom(T, R);

//     // Compute pose: perturbation of the initial pose
//     vpHomogeneousMatrix cMo_i = cMo_initial * delta_i;

//     // Render
//     if (moteur->continueRendering())
//         moteur->display(&cMo_i);
//     else
//     {
//         ROS_INFO("probleme rendu");
//         exit(12);
//     }

//     // Get image and save
//     kf_omni.kfresize(cam_omni.width, cam_omni.height);
//     moteur->getInternalImage(kf_omni.I);
//     moteur->getInternalImageZ(kf_omni.Idepth);
//     vpImageConvert::convert(kf_omni.I, kf_I, true);

//     // Save to disk
//     std::ostringstream oss;
//     oss << "/home/yufan/Data/2025/0606/edges_" << std::setfill('0') << std::setw(3) << i << ".png";
//     cv::imwrite(oss.str(), kf_I);

//     std::cout << "Saved: " << oss.str() << std::endl;
//   }



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
int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;
  
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);  // disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
    ch = getchar();
  
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
  
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
  
    return 0;
}

char getch()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // turn off echo and canonical mode
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}

void esvo_Tracking::TrackingLoop()
{


moteur = new gcOgre(cam, cam_omni.width, cam_omni.height, "/home/yufan/Dependency/ogre-1.12.2/OgreConfigs/");
moteur->init(); // must be in same thread
moteur->loadPointCloud("objecttotrack", evsModelPath_);
moteur->setClipDistances(cam_omni.clip_near, cam_omni.clip_far);
rerender();
// // Start timing
// auto start = std::chrono::high_resolution_clock::now();

// for (int i = 0; i < 100; ++i)
// {
//   rerender();
// }

// // Stop timing
// auto end = std::chrono::high_resolution_clock::now();
// std::chrono::duration<double> duration = end - start;
// std::cout << "Time taken for 100 rerenders: " << duration.count() << " seconds" << std::endl;
// std::cout << "Average time per rerender: " << (duration.count() / 100.0) * 1000.0 << " ms" << std::endl;

// cv::waitKey(0);

// }
  ros::Rate r(tracking_rate_hz_);
  while(ros::ok())
  {
    if (kbhit())
    {
        char c = getch();
        if (c == 's') {
            ROS_INFO("Key 's' pressed. Exiting loop.");
            break;
        }
    }
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

        // std::cout<<"Times are "<< ref_.t_.toSec() <<" "<< refPCMap_.rbegin()->first.toSec()<<std::endl;
      if(ref_.t_.toSec() < refPCMap_.rbegin()->first.toSec())// new reference map arrived
      // if(1)
      
      {
        refDataTransferring();
        renderFlag = true;
      }
      if(renderFlag)
      {
        auto t_start = std::chrono::steady_clock::now();
        renderCtr ++;
        // std::cout<<"Render counter = "<<renderCtr<<std::endl;
        if(renderCtr == 30)
        {
          auto t_start = std::chrono::steady_clock::now();
          rerender();
          renderCtr = 0;
          auto t_end = std::chrono::steady_clock::now();
          double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
          std::cout << "[Timer] Code block took " << elapsed_ms << " ms.\n";
        } 
    } 
      // std::cout<<cur_.t_.toSec() - TS_history_.rbegin()->first.toSec()<<std::endl;
      // std::cout<<TS_history_.rbegin()->first.toSec()<<std::endl;
      if(cur_.t_.toSec() < TS_history_.rbegin()->first.toSec())// new observation arrived
      {
        // if(ref_.t_.toSec() >= TS_history_.rbegin()->first.toSec())
        if(ref_.t_.toSec() > TS_history_.rbegin()->first.toSec())
        {
          std::cout<<ref_.t_.toSec()<<" "<<TS_history_.rbegin()->first.toSec()<<std::endl;
          LOG(INFO) << "The time_surface observation should be obtained after the reference frame";
          exit(-1); 
        }
        if(!curDataTransferring())
        {
          std::cout<<"Case1--------------------------------"<<std::endl;
          continue;
        }
      }
      else
      {
        // std::cout<<"Case2"<<std::endl;
        continue;
      }
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
    LOG(INFO) << "[DEBUG] resultPath_ = [" << resultPath_ << "]";

    LOG(INFO) << "pose size: " << lPose_.size();
    LOG(INFO) << "refPCMap_.size(): " << refPCMap_.size() << ", TS_history_.size(): " << TS_history_.size();
    // saveTrajectory(resultPath_ + "result.txt");/home/yufan/Data/2025/0405/
    saveTrajectory("/home/yufan/Data/2025/0801/cn/result.txt");
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
  // if(ESVO_System_Status_ == "WORKING" || (ESVO_System_Status_ == "INITIALIZATION" && ets_ == WORKING))
  // {
  //   if(!getPoseAt(ref_.t_, ref_.tr_, dvs_frame_id_))
  //   {
  //     LOG(INFO) << "ESVO_System_Status_: " << ESVO_System_Status_ << ", ref_.t_: " << ref_.t_.toNSec();
  //     LOG(INFO) << "Logic error ! There must be a pose for the given timestamp, because mapping has been finished.";
  //     exit(-1);
  //     return false;
  //   }
  // }

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
  std::cout<<ref_.vPointXYZPtr_.size()<<".."<<std::endl;
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
    std::cout<<"Cur transform set to identity"<<std::endl;
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
  // std::cout<<"Called"<<std::endl;
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
esvo_Tracking::refreshDepth(cv::Mat& edge, cv::Mat& depth){
  int count_255 = cv::countNonZero(edge == 255);
  cv::Mat img = edge;
  std::cout << "Number of pixels with value 255: " << count_255 << std::endl;
  // std::cout<<img.type()<<std::endl;
  cv::cvtColor(img, img, CV_GRAY2BGR);
  // std::cout<<img<<std::endl;


  pc_->clear();
  pc_->reserve(5000);
  int ctr = 0;
  float thres = 80.0;
  for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            // if ((abs(kf.gradient.at<Vector2d>(i, j)[0] * kf.gradient.at<Vector2d>(i, j)[1]) > 10) && (kf.depth.at<double>(i, j) > 0))
            if (edge.at<uchar>(i,j) > thres )
            {
                double z = depth.at<double>(i,j);

                if (z == -1.0)
                {

                  // ctr ++;
                    bool found = false;
                    for (int di = -3; di <= 3 && !found; ++di)
                    {
                        for (int dj = -3; dj <= 3 && !found; ++dj)
                        {
                            int ni = i + di;
                            int nj = j + dj;
                            if (ni >= 0 && ni < depth.rows && nj >= 0 && nj < depth.cols)
                            {
                                double neighbor_z = depth.at<double>(ni, nj);
                                if (neighbor_z > 0)
                                {
                                    z = neighbor_z;
                                    found = true;
                                }
                            }
                        }
                    }
                    // If found, assign the new depth value
                    // if (found)
                    //     kf_depth.at<double>(i, j) = z;
                }

                // if(z>0)
                // std::cout<<z<<" ";
                //   continue;
                visualizor_.DrawPoint(abs(edge.at<uchar>(i,j)), 255, 40,  Eigen::Vector2d(j,i), img);
                Eigen::Vector3d p_world;
                Eigen::Vector2d p_cam(j, i);
                camSysPtr_->cam_left_ptr_->cam2World(p_cam, 1.0 / z, p_world);
                Eigen::Matrix<double, 4, 4> T_world_result = ref_.tr_.getTransformationMatrix();
                p_world = T_world_result.block<3,3>(0,0) * p_world + T_world_result.block<3,1>(0,3);

                // Eigen::Vector2d p_tmp;
                // camSysPtr_->cam_left_ptr_->world2Cam(p_world, p_tmp);
                // std::cout<<p_tmp<<std::endl;
                pcl::PointXYZI tpc = pcl::PointXYZI(0.0);
                tpc.x = p_world(0);
                tpc.y = p_world(1);
                tpc.z = p_world(2);
                tpc.intensity = (abs(edge.at<uchar>(i,j))-thres)/(255.0-thres);
                pc_->push_back(tpc);
                // std::cout<<p_cam<<p_world<<std::endl;
            }
        }
    }
  // cv::imwrite("/home/yufan/Data/2025/0606/img.png", img);
  edge_image_counter ++;
  // std::ostringstream oss;
  // oss << "/home/yufan/Data/2025/0708/cn/edges_" << std::setfill('0') << std::setw(3) << edge_image_counter << ".png";
  // cv::imwrite(oss.str(), edge);
  std_msgs::Header header;
  header.stamp = fake_time_;
  fake_time_ += ros::Duration(0.1);  // Add 1 second

  refPCMap_.emplace(header.stamp, pc_); 
  if(refPCMap_.size() > REF_HISTORY_LENGTH_)
  {
    auto it = refPCMap_.begin();
    refPCMap_.erase(it);
    // std::cout<<"Depth map refreshed!!"<<std::endl;
  }
  
  // std::cout<<refPCMap_.rbegin()->first.toSec()<<std::endl;
  // std::cout<<"The point set number is "<<ctr<<std::endl;
  // std::cout << point_set->header.stamp << std::endl;

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
  // static cv_bridge::CvImage cv_image;
  // cv_image.encoding = "mono8";
  // cv_image.image = kf_I.clone();
  // cv_image.header.stamp = cv_ptr_ps->header.stamp;
  // pointSet_pub_.publish(cv_image.toImageMsg());
  pointSet_pub_.publish(msg);
  // psFlag = false;
}

void 
esvo_Tracking::pointsetCallback(const sensor_msgs::ImageConstPtr &point_set)
{
  // std::cout<<"This called"<<std::endl;
  if(psFlag == false)
    return;
  cv::Mat img;
  cv_ptr_ps = cv_bridge::toCvCopy(point_set, sensor_msgs::image_encodings::MONO8);
  cv_ptr_ps->image.copyTo(img);
  img = edges;

  int count_255 = cv::countNonZero(img == 255);

  std::cout << "Number of pixels with value 255: " << count_255 << std::endl;
  // std::cout<<img.type()<<std::endl;
  cv::cvtColor(img, img, CV_GRAY2BGR);
  // std::cout<<img<<std::endl;
  int ctr = 0;
  for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            // if ((abs(kf.gradient.at<Vector2d>(i, j)[0] * kf.gradient.at<Vector2d>(i, j)[1]) > 10) && (kf.depth.at<double>(i, j) > 0))
            if (edges.at<uchar>(i,j) == 255 )
            {
                double z = kf_depth.at<double>(i,j);

                if (z == -1.0)
                {

                  // ctr ++;
                    bool found = false;
                    for (int di = -3; di <= 3 && !found; ++di)
                    {
                        for (int dj = -3; dj <= 3 && !found; ++dj)
                        {
                            int ni = i + di;
                            int nj = j + dj;
                            if (ni >= 0 && ni < kf_depth.rows && nj >= 0 && nj < kf_depth.cols)
                            {
                                double neighbor_z = kf_depth.at<double>(ni, nj);
                                if (neighbor_z > 0)
                                {
                                    z = neighbor_z;
                                    found = true;
                                }
                            }
                        }
                    }
                    // If found, assign the new depth value
                    // if (found)
                    //     kf_depth.at<double>(i, j) = z;
                }

                if(z>0)
                std::cout<<z<<" ";
                //   continue;
                visualizor_.DrawPoint(1.0 / z, 1.0 / 0.3, 1.0 / mMax,  Eigen::Vector2d(j,i), img);
                Eigen::Vector3d p_world;
                Eigen::Vector2d p_cam(j, i);
                camSysPtr_->cam_left_ptr_->cam2World(p_cam, 1.0 / z, p_world);
                // Eigen::Vector2d p_tmp;
                // camSysPtr_->cam_left_ptr_->world2Cam(p_world, p_tmp);
                // std::cout<<p_tmp<<std::endl;
                // pc_->push_back(pcl::PointXYZ(p_world(0), p_world(1), p_world(2)));
                // std::cout<<p_cam<<p_world<<std::endl;
            }
        }
    }
  
  refPCMap_.emplace(cv_ptr_ps->header.stamp, pc_); 
  // std::cout<<refPCMap_.size()<<std::endl;
  // std::cout<<"The point set number is "<<ctr<<std::endl;
  std_msgs::Header header;
  header.stamp = cv_ptr_ps->header.stamp;
  std::cout << point_set->header.stamp << std::endl;

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
  size_t num_to_save = std::min(lPose_.size(), lTimestamp_.size());

  auto result_it = lPose_.begin();
  auto ts_it = lTimestamp_.begin();
  
  for (size_t i = 0; i < num_to_save; ++i, ++result_it, ++ts_it)
  {
      Eigen::Matrix3d Rwc_result = result_it->block<3,3>(0,0);
      Eigen::Vector3d twc_result = result_it->block<3,1>(0,3);
      Eigen::Quaterniond q(Rwc_result);
  
      f << *ts_it << " " << std::setprecision(9)
        << twc_result.transpose() << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
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

void esvo_Tracking::rerender()
{
// // Translation
// double tx = -0.080910067;
// double ty = -0.003411980;
// double tz = -0.007450045;

// // Quaternion (x, y, z, w)
// double qx = -0.028764999;
// double qy = -0.029430876;
// double qz =  0.026615042;
// double qw =  0.998798297;

// // Convert quaternion to rotation matrix using Eigen
// Eigen::Quaterniond q(qw, qx, qy, qz);
// Eigen::Matrix3d R = q.normalized().toRotationMatrix();

// // Fill into vpHomogeneousMatrix
// vpHomogeneousMatrix T;

// for (unsigned int i = 0; i < 3; ++i)
// {
//   for (unsigned int j = 0; j < 3; ++j)
//     T[i][j] = R(i, j);
// }

// T[0][3] = tx;
// T[1][3] = ty;
// T[2][3] = tz;
// T[3][3] = 1;

Eigen::Matrix4d curr_eigenMat = cur_.tr_.getTransformationMatrix();
// std::cout<<edge_image_counter<<"  ..............."<<std::endl;

vpHomogeneousMatrix T_rel;
for (unsigned int i = 0; i < 4; ++i)
  for (unsigned int j = 0; j < 4; ++j)
    T_rel[i][j] = curr_eigenMat(i, j);
// std::cout<<T_rel<<std::endl;
// cMo = T_rel.inverse() * c0Mo;
cMo = T_rel.inverse() * c0Mo;
// cur_.tr_.setIdentity();
ref_.tr_=cur_.tr_;
// prev_eigenMat = curr_eigenMat;

if (moteur->continueRendering())
  moteur->display(&cMo);

  kf_omni.kfresize(cam_omni.width, cam_omni.height);
  moteur->getInternalImage(kf_omni.I);
  moteur->getInternalImageZ(kf_omni.Idepth);

  // Fast depth conversion
  kf_depth = cv::Mat::zeros(cam_omni.height, cam_omni.width, CV_64FC1);
  // kf_depth = cv::Mat(cam_omni.height, cam_omni.width, CV_64FC1);
  for (int i = 0; i < cam_omni.height; ++i)
  {
    for (int j = 0; j < cam_omni.width; ++j)
    {
      kf_depth.at<double>(i, j) = static_cast<double>(kf_omni.Idepth[i][j]);
    }
  }

  cv::minMaxLoc(kf_depth, &mMin, &mMax, &minP, &maxP);

  vpImageConvert::convert(kf_omni.I, kf_I, true);
  cv::GaussianBlur(kf_I, blurred, cv::Size(3, 3), 1.0);
  // cv::Canny(kf_I, edges, 200, 600);

  cv::Sobel(kf_I, edges, -1 ,1, 1);

  // Fast benchmark-only depth visualization
  refreshDepth(edges, kf_depth);
}


}// namespace esvo_core