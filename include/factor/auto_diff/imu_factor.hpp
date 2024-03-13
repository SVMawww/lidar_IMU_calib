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

    size_t R_offset;  // should be zero if not estimate time offset
    double u;
    spline_meta_.ComputeSplineIndex(imu_data_.timestamp, R_offset, u);

    Tangent rot_vel;
    basalt::CeresSplineHelper<_N>::template evaluate_lie<T, Sophus::SO3>(
        sKnots + R_offset, u, inv_dt_, nullptr, &rot_vel, 
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
}
