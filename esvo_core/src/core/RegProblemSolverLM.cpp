#include <esvo_core/core/RegProblemSolverLM.h>
#include <esvo_core/tools/cayley.h>

namespace esvo_core
{
namespace core
{
RegProblemSolverLM::RegProblemSolverLM(
  esvo_core::CameraSystem::Ptr &camSysPtr,
  shared_ptr<RegProblemConfig> &rpConfigPtr,
  esvo_core::core::RegProblemType rpType,
  size_t numThread):
  camSysPtr_(camSysPtr),
  rpConfigPtr_(rpConfigPtr),
  rpType_(rpType),
  NUM_THREAD_(numThread),
  bPrint_(false),
  bVisualize_(true)
{
  if(rpType_ == REG_NUMERICAL)
  {
    numDiff_regProblemPtr_ =
      std::make_shared<Eigen::NumericalDiff<RegProblemLM> >(camSysPtr_, rpConfigPtr_, NUM_THREAD_);
  }
  else if(rpType_ == REG_ANALYTICAL)
  {
    regProblemPtr_ = std::make_shared<RegProblemLM>(camSysPtr_, rpConfigPtr_, NUM_THREAD_);
  }
  else
  {
    LOG(ERROR) << "Wrong Registration Problem Type is assigned!!!";
    exit(-1);
  }
  z_min_ = 1.0 / rpConfigPtr_->invDepth_max_range_;
  z_max_ = 1.0 / rpConfigPtr_->invDepth_min_range_;

  lmStatics_.nPoints_ = 0;
  lmStatics_.nfev_ = 0;
  lmStatics_.nIter_ = 0;
  
}

RegProblemSolverLM::~RegProblemSolverLM()
{}

bool RegProblemSolverLM::resetRegProblem(RefFrame* ref, CurFrame* cur)
{
  if(cur->numEventsSinceLastObs_ < rpConfigPtr_->MIN_NUM_EVENTS_)
  {
    LOG(INFO) << "resetRegProblem RESET fails for no enough events coming in.";
    LOG(INFO) << "However, the system remains to work.";
  }
  if( ref->vPointXYZPtr_.size() < rpConfigPtr_->BATCH_SIZE_ )
  {
    // std::cout<<ref->vPointXYZPtr_.size()<<" "<<rpConfigPtr_->BATCH_SIZE_<<std::endl;
    // LOG(INFO) << "resetRegProblem RESET fails for no enough point cloud in the local map. "<<ref->vPointXYZPtr_.size();
    // LOG(INFO) << "The system will be re-initialized";
    // return false;
    rpConfigPtr_->BATCH_SIZE_ = ref->vPointXYZPtr_.size();
  }
  //  LOG(INFO) << "resetRegProblem RESET succeeds.";
  if(rpType_ == REG_NUMERICAL)
  {
    numDiff_regProblemPtr_->setProblem(ref, cur, false);
//    LOG(INFO) << "numDiff_regProblemPtr_->setProblem(ref, cur, false) -----------------";
  }
  if(rpType_ == REG_ANALYTICAL)
  {
    regProblemPtr_->setProblem(ref, cur, true);
//    LOG(INFO) << "regProblemPtr_->setProblem(ref, cur, true) -----------------";
  }

  lmStatics_.nPoints_ = 0;
  lmStatics_.nfev_ = 0;
  lmStatics_.nIter_ = 0;
  return true;
}

bool RegProblemSolverLM::solve_numerical()
{
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<RegProblemLM>, double> lm(*numDiff_regProblemPtr_.get());
  lm.resetParameters();
  lm.parameters.ftol = 1e-3;
  lm.parameters.xtol = 1e-3;
  lm.parameters.maxfev = rpConfigPtr_->MAX_ITERATION_ * 8;

  size_t iteration = 0;
  size_t nfev = 0;
  while(true)
  {
    if(iteration >= rpConfigPtr_->MAX_ITERATION_)
      break;
    numDiff_regProblemPtr_->setStochasticSampling(
      (iteration % numDiff_regProblemPtr_->numBatches_) * rpConfigPtr_->BATCH_SIZE_, rpConfigPtr_->BATCH_SIZE_);
    Eigen::VectorXd x(6);
    x.fill(0.0);
    if(lm.minimizeInit(x) == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
    {
      LOG(ERROR) << "ImproperInputParameters for LM (Tracking)." << std::endl;
      return false;
    }

    Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeOneStep(x);
    numDiff_regProblemPtr_->addMotionUpdate(2*x);

    iteration++;
    nfev += lm.nfev;

    /*************************** Visualization ************************/
    if(bVisualize_)// will slow down the tracker's performance a little bit
    {
      size_t width = camSysPtr_->cam_left_ptr_->width_;
      size_t height = camSysPtr_->cam_left_ptr_->height_;
      cv::Mat reprojMap_left = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
      cv::eigen2cv(numDiff_regProblemPtr_->cur_->pTsObs_->TS_negative_left_, reprojMap_left);
      reprojMap_left.convertTo(reprojMap_left, CV_8UC1);
      cv::cvtColor(reprojMap_left, reprojMap_left, CV_GRAY2BGR);

      // project 3D points to current frame
      Eigen::Matrix3d R_cur_ref =  numDiff_regProblemPtr_->R_.transpose();
      Eigen::Vector3d t_cur_ref = -numDiff_regProblemPtr_->R_.transpose() * numDiff_regProblemPtr_->t_;

      size_t numVisualization = std::min(numDiff_regProblemPtr_->ResItems_.size(), (size_t)2000);
      for(size_t i = 0; i < numVisualization; i++)
      {
        ResidualItem & ri = numDiff_regProblemPtr_->ResItems_[i];
        Eigen::Vector3d p_3D = R_cur_ref * ri.p_ + t_cur_ref;
        Eigen::Vector2d p_img_left;
        camSysPtr_->cam_left_ptr_->world2Cam(p_3D, p_img_left);
        double z = ri.p_[2];
        visualizor_.DrawPoint(1.0 / z, 1.0 / z_min_, 1.0 / z_max_,
                              Eigen::Vector2d(p_img_left(0), p_img_left(1)), reprojMap_left);
      }
      std_msgs::Header header;
      header.stamp = numDiff_regProblemPtr_->cur_->t_;
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", reprojMap_left).toImageMsg();
      reprojMap_pub_->publish(msg);
    }
    /*************************** Visualization ************************/
    if(status == 2 || status == 3)
      break;
  }
//  LOG(INFO) << "LM Finished ...................";
  numDiff_regProblemPtr_->setPose();
  lmStatics_.nPoints_ = numDiff_regProblemPtr_->numPoints_;
  lmStatics_.nfev_ = nfev;
  lmStatics_.nIter_ = iteration;
  return 0;
}

bool RegProblemSolverLM::solve_analytical()
{
  Eigen::LevenbergMarquardt<RegProblemLM, double> lm(*regProblemPtr_.get());
  lm.resetParameters();
  lm.parameters.ftol = 1e-3;
  lm.parameters.xtol = 1e-3;
  lm.parameters.maxfev = rpConfigPtr_->MAX_ITERATION_ * 8;

  size_t iteration = 0;
  size_t big_iteration = 0;
  size_t nfev = 0;
  Eigen::Matrix3d skew_R_rel;
  Eigen::Vector3d dt;
  while(true)
  {
    // startWhile:
    // std::cout<<"w"<<std::endl;
    if(iteration >= rpConfigPtr_->MAX_ITERATION_)
      break;
    regProblemPtr_->setStochasticSampling(
      (iteration % regProblemPtr_->numBatches_) * rpConfigPtr_->BATCH_SIZE_, rpConfigPtr_->BATCH_SIZE_);
    Eigen::VectorXd x(6);
    x.fill(0.0);
    if(lm.minimizeInit(x) == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
    {
      LOG(ERROR) << "ImproperInputParameters for LM (Tracking)." << std::endl;
      return false;
    }
    Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeOneStep(x);
    // std::cout<<"Poses are "<<x<<std::endl;
    regProblemPtr_->addMotionUpdate(2.0*x);

    // Eigen::Vector3d dc = x.block<3,1>(0,0);
    // dt = x.block<3,1>(3,0);
    // // add rotation
    // Eigen::Matrix3d dR = tools::cayley2rot(dc);
    // skew_R_rel = (dR - dR.transpose()) / 2.0;
    iteration++;
    /////////////////////////////////////////////////////////////////
    // big_iteration = std::floor((iteration - 1) / regProblemPtr_->numBatches_)+1;

    // size_t width = camSysPtr_->cam_left_ptr_->width_;
    // size_t height = camSysPtr_->cam_left_ptr_->height_;    

    // cv::Mat reprojMap_left = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
    // // std::cout<<"0"<<std::endl;
    // // if(regProblemPtr_->cur_->pTsObs_->TS_negative_left_.empty())
    // //   continue;
    //     if (regProblemPtr_->cur_->pTsObs_->TS_negative_left_.rows() != height ||
    //     regProblemPtr_->cur_->pTsObs_->TS_negative_left_.cols() != width) {
    //     throw std::runtime_error("Dimension mismatch between TS_negative_left_ and reprojMap_left");
    // }
    // cv::eigen2cv(regProblemPtr_->cur_->pTsObs_->TS_negative_left_, reprojMap_left);
    // reprojMap_left.convertTo(reprojMap_left, CV_8UC1);
    // cv::cvtColor(reprojMap_left, reprojMap_left, CV_GRAY2BGR);
    // // std::cout<<"1"<<std::endl;

    // // project 3D points to current frame
    // Eigen::Matrix3d R_cur_ref =  regProblemPtr_->R_.transpose();
    // Eigen::Vector3d t_cur_ref = -regProblemPtr_->R_.transpose() * regProblemPtr_->t_;

    // size_t numVisualization = std::min(regProblemPtr_->ResItems_.size(), (size_t)3000);
    // // std::cout<<"2"<<std::endl;
    // for(size_t i = 0; i < numVisualization; i++)
    // {
    //   ResidualItem & ri = regProblemPtr_->ResItems_[i];
    //   Eigen::Vector3d p_3D = R_cur_ref * ri.p_ + t_cur_ref;
    //   Eigen::Vector2d p_img_left;
    //   camSysPtr_->cam_left_ptr_->world2Cam(p_3D, p_img_left);
    //   double z = ri.p_[2];
    //   if(p_img_left(0) < 0 || p_img_left(1) < 0 || p_img_left(0) > 640 || p_img_left(1) > 480)
    //     goto startWhile;
    //   visualizor_.DrawPoint(1.0 / z, 1.0 / z_min_, 1.0 / z_max_,
    //                         Eigen::Vector2d(p_img_left(0), p_img_left(1)), reprojMap_left);
    // }
    // std::cout<<"3"<<std::endl;

    // std::stringstream ss;
    // ss << big_iteration;
    // std::string filename = "/home/yufan/Data/2024/0910/exp2/" + ss.str() + ".jpg";
    // std::cout<<filename<<std::endl;
    // if(!reprojMap_left.empty())
    //   cv::imwrite(filename, reprojMap_left);
    // std::cout<<filename<<std::endl;
    ///////////////////////////////////////////////////////////////
    // std::cout<<"3"<<std::endl;
    nfev += lm.nfev;
    if(status == 2 || status == 3){

      break;
    }
  }
  // std::cout << "Current time: " << regProblemPtr_->cur_->t_.sec << " seconds and " 
  //             << regProblemPtr_->cur_->t_.nsec << " nanoseconds" << std::endl;

  // std::cout<<"sync time = "<<regProblemPtr_->cur_->t_<<std::endl;
  // cv::waitKey(0);
  // This is the 6-D velocity to publish
  // double delta_t = 0.01;
  double delta_t = 1;
  skew_R_rel = (regProblemPtr_->R_ - regProblemPtr_->R_.transpose()) / 2.0;
  geometry_msgs::Twist msg_vel;
  msg_vel.angular.x = -skew_R_rel(2,1)/delta_t;
  msg_vel.angular.y = -skew_R_rel(0,2)/delta_t;
  msg_vel.angular.z = -skew_R_rel(1,0)/delta_t;
  msg_vel.linear.x = -regProblemPtr_->t_[0]/delta_t;
  msg_vel.linear.y = -regProblemPtr_->t_[1]/delta_t;
  msg_vel.linear.z = -regProblemPtr_->t_[2]/delta_t;
  evs_pub_.publish(msg_vel);


  /*************************** Visualization ************************/
  if(bVisualize_) // will slow down the tracker a little bit
  {
    size_t width = camSysPtr_->cam_left_ptr_->width_;
    size_t height = camSysPtr_->cam_left_ptr_->height_;
    cv::Mat reprojMap_left = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
    cv::eigen2cv(regProblemPtr_->cur_->pTsObs_->TS_negative_left_, reprojMap_left);
    reprojMap_left.convertTo(reprojMap_left, CV_8UC1);
    cv::cvtColor(reprojMap_left, reprojMap_left, CV_GRAY2BGR);

    // project 3D points to current frame
    Eigen::Matrix3d R_cur_ref =  regProblemPtr_->R_.transpose();
    Eigen::Vector3d t_cur_ref = -regProblemPtr_->R_.transpose() * regProblemPtr_->t_;

    size_t numVisualization = std::min(regProblemPtr_->ResItems_.size(), (size_t)3000);
    for(size_t i = 0; i < numVisualization; i++)
    {
      ResidualItem & ri = regProblemPtr_->ResItems_[i];
      Eigen::Vector3d p_3D = R_cur_ref * ri.p_ + t_cur_ref;
      Eigen::Vector2d p_img_left;
      camSysPtr_->cam_left_ptr_->world2Cam(p_3D, p_img_left);
      double z = ri.p_[2];
      visualizor_.DrawPoint(1.0 / z, 1.0 / z_min_, 1.0 / z_max_,
                            Eigen::Vector2d(p_img_left(0), p_img_left(1)), reprojMap_left);
    }
      // cv::imwrite("/home/yufan/Data/2024/0910/exp2/reproj.jpg", reprojMap_left);
  
    std_msgs::Header header;
    header.stamp = regProblemPtr_->cur_->t_;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", reprojMap_left).toImageMsg();
    reprojMap_pub_->publish(msg);
  }
  /*************************** Visualization ************************/

  regProblemPtr_->setPose();
  lmStatics_.nPoints_ = regProblemPtr_->numPoints_;
  lmStatics_.nfev_ = nfev;
  lmStatics_.nIter_ = iteration;
  return 0;
}

void RegProblemSolverLM::setRegPublisher(
  image_transport::Publisher* reprojMap_pub)
{
  reprojMap_pub_ = reprojMap_pub;
}

void RegProblemSolverLM::setevsPublisher(
  ros::Publisher evs_pub)
{
  evs_pub_ = evs_pub;
}
}//namespace core
}//namespace esvo_core
