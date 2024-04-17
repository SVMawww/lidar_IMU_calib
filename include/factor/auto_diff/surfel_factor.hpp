#ifndef SURFEL_FACTOR_HPP
#define SURFEL_FACTOR_HPP

#include <basalt/spline/ceres_spline_helper.h>
#include <basalt/spline/ceres_spline_helper_jet.h>
#include <basalt/spline/spline_segment.h>
#include <ceres/ceres.h>
// #include "utils/lidar_model.h"
#include <ceres/rotation.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace licalib{
template <int _N>
class surfelFactorWithRelative {

private:
  // std::shared_ptr<Lidar> lidar_;
  // size_t spline_meta__NumParameters;
  double weight_;
  double map_time_;
  double lidar_time_;
  Eigen::Vector3d lidar_point_;
  // Sophus::SE3d T_I0toG_;
  // Sophus::SE3d T_IktoG_;
  basalt::SplineMeta<_N> spline_meta_;
  // double* plane_;
  double inv_dt_;
public:

surfelFactorWithRelative(double maptime, double lidar_time, double weight, 
            Eigen::Vector3d lidarpoint,
            const basalt::SplineMeta<_N>& spline_meta) 
      : spline_meta_(spline_meta),
      weight_(weight), lidar_point_(lidarpoint),
      map_time_(maptime), lidar_time_(lidar_time) {
        inv_dt_ = 1.0 / spline_meta.segments.begin()->dt;
      }

template <typename T>
bool operator()(const T* const* params, T* residual) const {
  using SO3T = Sophus::SO3<T>;
  using Vec3T = Eigen::Matrix<T, 3, 1>;
  // auto T_I0toG = T_I0toG_.cast<T>();
  // auto T_IktoG = T_IktoG_.cast<T>();
  size_t R_offset;
  size_t P_offset;
  T u;
  spline_meta_.ComputeSplineIndex(T(map_time_), R_offset, u);
  P_offset = R_offset + spline_meta_.NumParameters();
  SO3T R_I0ToG, R_IkToG;
  Vec3T p_I0ToG, p_IkToG;
  basalt::CeresSplineHelperJet<T, _N>::template evaluate_lie<Sophus::SO3>(
    params + R_offset, u, inv_dt_, &R_I0ToG
  );
  basalt::CeresSplineHelperJet<T, _N>::template evaluate<3, 0>(
    params + P_offset, u, inv_dt_, &p_I0ToG
  );
  spline_meta_.ComputeSplineIndex(T(lidar_time_), R_offset, u);
  P_offset = R_offset + spline_meta_.NumParameters();
  basalt::CeresSplineHelperJet<T, _N>::template evaluate_lie<Sophus::SO3>(
    params + R_offset, u, inv_dt_, &R_IkToG
  );
  basalt::CeresSplineHelperJet<T, _N>::template evaluate<3, 0>(
    params + P_offset, u, inv_dt_, &p_IkToG
  );
  size_t offset = 0;
  offset += 2 * spline_meta_.NumParameters();
  // auto lidar_param = params[offset];
  Eigen::Matrix<T, 3, 1> p_LinI = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(params[offset]);
  Eigen::Quaternion<T> q_LtoI = Eigen::Map<const Eigen::Quaternion<T>>(params[offset+1]);
  Eigen::Matrix<T, 3, 1> p_Lk = lidar_point_.cast<T>();
  Eigen::Matrix<T, 3, 1> p_I = q_LtoI * p_Lk + p_LinI;
  // Eigen::Matrix<T, 3, 1> p_temp = T_I0toG.so3().unit_quaternion().conjugate()*(T_IktoG.so3().unit_quaternion() * p_I + T_IktoG.translation() - T_I0toG.translation());
  Eigen::Matrix<T, 3, 1> p_temp = R_I0ToG.unit_quaternion().conjugate()*(R_IkToG.unit_quaternion() * p_I + p_IkToG - p_I0ToG);
  Eigen::Matrix<T, 3, 1> p_M = q_LtoI.conjugate() * (p_temp - p_LinI);
  offset += 2;
  // 3,one for vector3, one for quaternion, one for time_offset
  // time_offset deleted, now 2
  
  // const T* plane_cp = params[offset];
  Eigen::Matrix<T,3,1> Pi = Eigen::Map<const Eigen::Matrix<T,3,1>>(params[offset]);
  T plane_d = T(Pi.norm());
  T plane_norm[3];
  plane_norm[0] = T(Pi[0])/plane_d;
  plane_norm[1] = T(Pi[1])/plane_d;
  plane_norm[2] = T(Pi[2])/plane_d;

  T dist = ceres::DotProduct(plane_norm, p_M.data()) - plane_d;
  Eigen::Map<Eigen::Matrix<T,1,1>> r(residual);
  Eigen::Matrix<T, 1, 1> error(dist);
  r = T(weight_) * error;
  // std::cout << "in fucking factor fucks fuck off!" << std::endl;
  return true;
}

};
}

#endif 