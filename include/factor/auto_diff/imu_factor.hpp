// rewrite b-spline computing using basalt
// libaorun 24.1.7


#include <basalt/spline/ceres_spline_helper.h>
#include <basalt/spline/ceres_spline_helper_jet.h>
#include <basalt/spline/spline_segment.h>
#include <ceres/ceres.h>
#include "utils/dataset_reader.h"
#include <sophus/so3.hpp>

namespace licalib {

template <int _N>
class gyroSO3ConstBiasFactor : public basalt::CeresSplineHelper<_N> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  gyroSO3ConstBiasFactor(const IO::IMUData& imu_data,
                                  const basalt::SplineMeta<_N>& spline_meta,
                                  double weight)
      : imu_data_(imu_data), spline_meta_(spline_meta), weight_(weight) {
    inv_dt_ = 1.0 / spline_meta_.segments.begin()->dt;
  }

  template <class T>
  bool operator()(const T* const* sKnots, T* sResiduals) const {
    using Tangent = typename Sophus::SO3<T>::Tangent;
    Eigen::Map<Tangent> residuals(sResiduals);
    Sophus::SO3<T> temp;
    size_t R_offset;  // should be zero if not estimate time offset
    double u;
    spline_meta_.ComputeSplineIndex(imu_data_.timestamp, R_offset, u);

    Tangent rot_vel;
    basalt::CeresSplineHelper<_N>::template evaluate_lie<T, Sophus::SO3>(
        sKnots + R_offset, u, inv_dt_, &temp, &rot_vel, 
        nullptr, nullptr);

    size_t Kont_offset = spline_meta_.NumParameters();
    Eigen::Map<Tangent const> const bias(sKnots[Kont_offset]);

    residuals = rot_vel - imu_data_.gyro.template cast<T>() + bias;
    residuals = T(weight_) * residuals;

    return true;
  }

 private:
  IO::IMUData imu_data_;
  basalt::SplineMeta<_N> spline_meta_;
  double weight_;
  double inv_dt_;
};

template <int _N>
class AccelVec3WithConstBiasFactor : public basalt::CeresSplineHelper<_N> {

};

template <int _N>
class GyroAcceWithBiasFactor : public basalt::CeresSplineHelper<_N> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GyroAcceWithBiasFactor(const IO::IMUData& imu_data,
                                 const basalt::SplineMeta<_N>& spline_meta,
                                 double gyro_weight, double acce_weight)
      : imu_data_(imu_data),
        spline_meta_(spline_meta),
        gyro_weight_(gyro_weight),
        acce_weight_(acce_weight) {
    inv_dt_ = 1.0 / spline_meta_.segments.begin()->dt;
  }

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec2T = Eigen::Matrix<T, 2, 1>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Vec6T = Eigen::Matrix<T, 6, 1>;
    using SO3T = Sophus::SO3<T>;
    using Tangent = typename Sophus::SO3<T>::Tangent;

    Eigen::Map<Vec6T> residuals(sResiduals);

    size_t R_offset;  // should be zero if not estimate time offset
    size_t P_offset;
    double u;
    spline_meta_.ComputeSplineIndex(imu_data_.timestamp, R_offset, u);
    P_offset = R_offset + spline_meta_.NumParameters();

    SO3T R_w_i;
    Tangent rot_vel;
    basalt::CeresSplineHelper<_N>::template evaluate_lie<T, Sophus::SO3>(
        sKnots + R_offset, u, inv_dt_, &R_w_i, &rot_vel);

    Vec3T accel_w;
    basalt::CeresSplineHelper<_N>::template evaluate<T, 3, 2>(sKnots + P_offset, u,
                                                      inv_dt_, &accel_w);

    size_t Kont_offset = 2 * spline_meta_.NumParameters();
    Eigen::Map<const Vec3T> gyro_bias(sKnots[Kont_offset]);
    Eigen::Map<const Vec3T> acce_bias(sKnots[Kont_offset + 1]);
    Vec3T gravity(T(0), T(0), T(9.8));

    Vec3T gyro_residuals =
        rot_vel - imu_data_.gyro.template cast<T>() + gyro_bias;
    Vec3T acce_residuals = R_w_i.inverse() * (accel_w + gravity) -
                           imu_data_.accel.template cast<T>() + acce_bias;

    residuals.template block<3, 1>(0, 0) = T(gyro_weight_) * gyro_residuals;
    residuals.template block<3, 1>(3, 0) = T(acce_weight_) * acce_residuals;

    return true;
  }

 private:
  IO::IMUData imu_data_;
  basalt::SplineMeta<_N> spline_meta_;
  double gyro_weight_;
  double acce_weight_;
  double inv_dt_;
};


}
