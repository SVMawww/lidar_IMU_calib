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
  Eigen::Vector3d lidar_point_;
  Sophus::SE3d T_I0toG_;
  Sophus::SE3d T_IktoG_;
  // double* plane_;
public:

surfelFactorWithRelative(// std::shared_ptr<Lidar> inputlidar, //size_t splinemetanums, 
            double maptime, double weight, Eigen::Vector3d lidarpoint,
            Sophus::SE3d& ti0tog, Sophus::SE3d& tiktog) 
                        : // lidar_(inputlidar), //spline_meta__NumParameters(splinemetanums),
                          weight_(weight), lidar_point_(lidarpoint),
                          map_time_(maptime),
                          T_I0toG_(ti0tog), T_IktoG_(tiktog)
                          {}

template <typename T>
bool operator()(const T* const* params, T* residual) const {
  auto T_I0toG = T_I0toG_.cast<T>();
  auto T_IktoG = T_IktoG_.cast<T>();
  size_t offset = 0;

  auto lidar_param = params[offset];
  Eigen::Matrix<T, 3, 1> p_LinI = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(lidar_param);
  Eigen::Quaternion<T> q_LtoI = Eigen::Map<const Eigen::Quaternion<T>>(lidar_param+1);
  Eigen::Matrix<T, 3, 1> p_Lk = lidar_point_.cast<T>();
  Eigen::Matrix<T, 3, 1> p_I = q_LtoI * p_Lk + p_LinI;
  Eigen::Matrix<T, 3, 1> p_temp = T_I0toG.so3().unit_quaternion().conjugate()*(T_IktoG.so3().unit_quaternion() * p_I + T_IktoG.translation() - T_I0toG.translation());
  Eigen::Matrix<T, 3, 1> p_M = q_LtoI.conjugate() * (p_temp - p_LinI);
  offset += 2;
  // 3,one for vector3, one for quaternion, one for time_offset
  // time_offset deleted, now 2
  const T* plane_cp = params[offset];
  Eigen::Matrix<T,3,1> Pi = Eigen::Map<const Eigen::Matrix<T,3,1>>(plane_cp);
  T plane_d = T(Pi.norm());
  T plane_norm[3];
  plane_norm[0] = T(Pi[0])/plane_d;
  plane_norm[1] = T(Pi[1])/plane_d;
  plane_norm[2] = T(Pi[2])/plane_d;

  T dist = ceres::DotProduct(plane_norm, p_M.data()) - plane_d;
  Eigen::Map<Eigen::Matrix<T,1,1>> r(residual);
  Eigen::Matrix<T, 1, 1> error;
  error[0] = dist;
  r = T(weight_) * error;
  return true;
}

};
}

#endif 