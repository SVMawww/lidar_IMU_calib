// rewrite b-spline computing using basalt
// libaorun 24.1.7

#include "core/trajectory_estimator.hpp"

 
namespace licalib {
//   template<int N>
// // modify from AddIMUGyroMeasurement
// void TrajEstimator<N>::addConstBiasGyroMeasurements(const std::vector<IO::IMUData>& imu_datas,
//                                                   double gyro_weight){
//   for(auto& imu_data : imu_datas){
//     basalt::SplineMeta<N> spline_meta;
//     TrajPtr->CalculateSplineMeta({{imu_data.timestamp, imu_data.timestamp}},
//                                 spline_meta);
//     using Functor = gyroSO3ConstBiasFactor<N>;
//     auto functor = std::make_unique<Functor>(imu_data, spline_meta, gyro_weight);
//     auto cost_function = std::make_unique<ceres::DynamicAutoDiffCostFunction<Functor>>(functor.get());
//   // Functor* functor = new Functor(imu_data, spline_meta, gyro_weight);
//   // auto* cost_function =
//       // new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

//   /// add so3 knots
//     for (int i = 0; i < spline_meta.NumParameters(); i++) {
//       cost_function->AddParameterBlock(4);
//     }
//     cost_function->AddParameterBlock(3);  // gyro bias

//     cost_function->SetNumResiduals(3);

//     std::vector<double*> vec;
//     AddControlPoints(spline_meta, vec);

//     vec.emplace_back(calib_param_->gyro_bias.data());
//     problem_->AddParameterBlock(calib_param_->gyro_bias.data(), 3);
//     problem_->SetParameterBlockConstant(calib_param_->gyro_bias.data());

//     problem_->AddResidualBlock(cost_function.get(), NULL, vec);

//   }
// }

// template <int N>
// void TrajEstimator<N>::AddControlPoints(
//     const basalt::SplineMeta<N>& spline_meta, std::vector<double*>& vec,
//     bool addPosKont) {
//   for (auto const& seg : spline_meta.segments) {
//     size_t start_idx = TrajPtr->computeTIndex(seg.t0 + 1e-9).second;
//     for (size_t i = start_idx; i < (start_idx + seg.NumParameters()); ++i) {
//       if (addPosKont) {
//         vec.emplace_back(TrajPtr->getKnotPos(i).data());
//         problem_->AddParameterBlock(TrajPtr->getKnotPos(i).data(), 3);
//       } else {
//         vec.emplace_back(TrajPtr->getKnotSO3(i).data());
//         problem_->AddParameterBlock(TrajPtr->getKnotSO3(i).data(), 4,
//                                     local_parameterization);
//       }
//       // if (IsLocked() || (fixed_control_point_index_ >= 0 &&
//       //                    i <= fixed_control_point_index_)) {
//       //   problem_->SetParameterBlockConstant(vec.back());
//       // }
//     }
//   }
// }


// template <int N>
// ceres::Solver::Summary TrajEstimator<N>::Solve(int max_iterations,
//                                                       bool progress,
//                                                       int num_threads) {
//   ceres::Solver::Options options;

//   options.minimizer_type = ceres::TRUST_REGION;
//   //  options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
//   //  options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
//   options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//   //    options.trust_region_strategy_type = ceres::DOGLEG;
//   //    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

//   //    options.linear_solver_type = ceres::SPARSE_SCHUR;
//   options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

//   options.minimizer_progress_to_stdout = progress;

//   if (num_threads < 1) {
//     num_threads = std::thread::hardware_concurrency();
//   }
//   options.num_threads = num_threads;
//   options.max_num_iterations = max_iterations;

//   if (callbacks_.size() > 0) {
//     for (auto& cb : callbacks_) {
//       options.callbacks.push_back(cb.get());
//     }

//     if (callback_needs_state_) options.update_state_every_iteration = true;
//   }

//   ceres::Solver::Summary summary;
//   ceres::Solve(options, problem_.get(), &summary);

//   //  getCovariance();
//   return summary;
// }


}