// // rewrite b-spline computing using basalt
// // libaorun 23.12.22

#include "core/traj_manager.h"


namespace licalib {

// template <int N>
// void TrajManager<N>::FeedIMUData(const IO::IMUData& imudata) {
//   this->imu_data_.emplace_back(imudata);
// }

// template <int N>
// void TrajManager<N>::initialSO3TrajWithGyro() {

  // assert(imu_data_.size() > 0 &&
//     "[initialSO3TrajWithGyro]: There's NO imu data for initialization.");
  
//   auto estimator_so3 = std::make_shared<TrajEstimator<N>>(TrajPtr, calib_param);
//   estimator_so3->addConstBiasGyroMeasurements(imu_data_, calib_param->global_opt_gyro_weight);
//   // fix the initial pose of trajectory
//   Eigen::AngleAxisd rotation_vector(0.0001, Eigen::Vector3d(0,0,1));
//   Eigen::Quaterniond q0 = Eigen::Quaterniond (rotation_vector.matrix());

//   TrajPtr->setKnotSO3(q0, 0);

//   ceres::Solver::Summary summary = estimator_so3->Solve(30, false, -1);
// }

// template <int N>
// decltype(auto) TrajManager<N>::evaluatePose(double time){
//   return TrajPtr->pose(time);
// }

// template <int N>
// CalibParamManager::Ptr TrajManager<N>::getCalibParamManager() const {
//   return calib_param;
// }
}

