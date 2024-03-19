// rewrite b-spline computing using basalt
// libaorun 24.1.6

#include <thread>

#include <ceres/ceres.h>
#include <ceres/covariance.h>

#include "utils/dataset_reader.h"
#include "core/basalt_traj.hpp"
#include "factor/auto_diff/imu_factor.hpp"
#include "core/calibration.hpp"
#include <basalt/spline/ceres_local_param.hpp>

namespace licalib {
  
template <int N>
class TrajEstimator{
  static ceres::Problem::Options DefaultProblemOptions(){
    ceres::Problem::Options options;
    options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    return options;
  }

public:
  TrajEstimator(std::shared_ptr<TrajSE3<N>> trajectory,
                      CalibParamManager::Ptr calib_param)
      : TrajPtr(trajectory), calib_param_(calib_param) {
    problem_ = std::make_shared<ceres::Problem>(DefaultProblemOptions());
    local_parameterization = new basalt::LieLocalParameterization<Sophus::SO3d>();
  }
  void addConstBiasGyroMeasurements(const std::vector<IO::IMUData>& imu_datas,
                                    double gyro_weight){
  for(auto& imu_data : imu_datas){
    basalt::SplineMeta<N> spline_meta;

    TrajPtr->CalculateSplineMeta({{imu_data.timestamp, imu_data.timestamp}},
                                spline_meta);
    using Functor = gyroSO3ConstBiasFactor<N>;
    // Functor* functor = new Functor(imu_data, spline_meta, gyro_weight);
    // auto* cost_function = new ceres::DynamicAutoDiffCostFunction<Functor>(functor);
    Functor* functor = new Functor(imu_data, spline_meta, gyro_weight);
    auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  // add so3 knots
 // b /root/li_calib/src/lidar_IMU_calib/include/core/trajectory_estimator.hpp:49
    for (int i = 0; i < spline_meta.NumParameters(); i++) {
      cost_function->AddParameterBlock(4);
    }

    cost_function->AddParameterBlock(3);  // gyro bias

    cost_function->SetNumResiduals(3);

    std::vector<double*> vec;
    AddControlPoints(spline_meta, vec, false);

    vec.emplace_back(calib_param_->gyro_bias.data());
    problem_->AddParameterBlock(calib_param_->gyro_bias.data(), 3);
    problem_->SetParameterBlockConstant(calib_param_->gyro_bias.data());

    problem_->AddResidualBlock(cost_function, NULL, vec);

  }
}
  
void AddControlPoints(
    const basalt::SplineMeta<N>& spline_meta, std::vector<double*>& vec,
    bool addPosKont) {
  for (auto const& seg : spline_meta.segments) {
    size_t start_idx = TrajPtr->computeTIndex(seg.t0 + 1e-9).second;
    for (size_t i = start_idx; i < (start_idx + seg.NumParameters()); ++i) {
      if (addPosKont) {
        vec.emplace_back(TrajPtr->getKnotPos(i).data());
        problem_->AddParameterBlock(TrajPtr->getKnotPos(i).data(), 3);
      } else {
        vec.emplace_back(TrajPtr->getKnotSO3(i).data());
        problem_->AddParameterBlock(TrajPtr->getKnotSO3(i).data(), 4,
                                    local_parameterization);
      }
      // if (IsLocked() || (fixed_control_point_index_ >= 0 &&
      //                    i <= fixed_control_point_index_)) {
      //   problem_->SetParameterBlockConstant(vec.back());
      // }
    }
  }
}

  ceres::Solver::Summary Solve(int max_iterations,
                               bool progress,
                               int num_threads) {
  ceres::Solver::Options options;

  options.minimizer_type = ceres::TRUST_REGION;
  //  options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  //  options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  //    options.trust_region_strategy_type = ceres::DOGLEG;
  //    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

  //    options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  options.minimizer_progress_to_stdout = progress;

  if (num_threads < 1) {
    num_threads = std::thread::hardware_concurrency();
  }
  options.num_threads = num_threads;
  options.max_num_iterations = max_iterations;

  if (callbacks_.size() > 0) {
    for (auto& cb : callbacks_) {
      options.callbacks.push_back(cb.get());
    }

    if (callback_needs_state_) options.update_state_every_iteration = true;
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);

  //  getCovariance();
  return summary;
}
private:
  std::shared_ptr<TrajSE3<N>> TrajPtr;
  std::shared_ptr<ceres::Problem> problem_;
  std::shared_ptr<CalibParamManager> calib_param_;
  ceres::LocalParameterization* local_parameterization;

  std::vector<std::unique_ptr<ceres::IterationCallback>> callbacks_;
  bool callback_needs_state_;
};

}