// rewrite b-spline computing using basalt
// libaorun 23.12.22


#pragma once



#include <basalt/spline/se3_spline.h>
#include "utils/dataset_reader.h"
#include "core/trajectory_estimator.hpp"


namespace licalib {

template <int N=4>
class TrajManager {
private:
  std::shared_ptr<TrajSE3<N>> TrajPtr;
  std::vector<IO::IMUData> imu_data_;
  std::shared_ptr<CalibParamManager> calib_param;
  double map_time_;
public:
  
  using Ptr = std::shared_ptr<TrajManager<4>>;
  TrajManager(double interval, double start_time) : map_time_(0) {
    TrajPtr = std::make_shared<TrajSE3<4>>(interval, start_time);
    calib_param = std::make_shared<CalibParamManager>();
  }

  std::shared_ptr<TrajSE3<N>> getTrajPtr() {return TrajPtr;}

  void initialSO3TrajWithGyro() {

  assert(imu_data_.size() > 0 &&
    "[initialSO3TrajWithGyro]: There's NO imu data for initialization.");

  auto estimator_so3 = std::make_shared<TrajEstimator<N>>(TrajPtr, calib_param);

  estimator_so3->addConstBiasGyroMeasurements(imu_data_, calib_param->global_opt_gyro_weight);
 // b /root/li_calib/src/lidar_IMU_calib/include/core/traj_manager.h:41
  // fix the initial pose of trajectory
  Eigen::AngleAxisd rotation_vector(0.0001, Eigen::Vector3d(0,0,1));
  Eigen::Quaterniond q0 = Eigen::Quaterniond (rotation_vector.matrix());

  TrajPtr->setKnotSO3(Sophus::SO3<double>(q0), 0);

  ceres::Solver::Summary summary = estimator_so3->Solve(30, false, -1);
}
  // void FeedIMUData(const IO::IMUData& imudata);
  void FeedIMUData(const IO::IMUData& imudata) {
  this->imu_data_.emplace_back(imudata);
}

  decltype(auto) evaluatePose(double time){
  return TrajPtr->pose(time);
}
  CalibParamManager::Ptr getCalibParamManager() const {
  return calib_param;
}

  bool evaluateLidarPose(double lidar_time, 
                         Eigen::Quaterniond& qL0ToG, 
                         Eigen::Vector3d& pL0InG) {
    if(TrajPtr->maxTime() <= lidar_time || TrajPtr->minTime() >= lidar_time) 
      return false;
    auto res = TrajPtr->EvaluatePose(lidar_time);
    qL0ToG = res.so3().unit_quaternion() * calib_param->q_LtoI;
    pL0InG = res.so3().unit_quaternion() * calib_param->p_LinI + res.translation();
    return true;
  }

  double get_map_time() {
    return map_time_;
  }

};


}
