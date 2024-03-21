// rewrite b-spline computing using basalt
// libaorun 23.12.22

#include <basalt/spline/so3_spline.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/spline/rd_spline.h>
#include <memory>

template <int N=4>
class TrajSE3 : public basalt::Se3Spline<N> {


private: 
  std::shared_ptr<basalt::So3Spline<N>> so3_traj;
  std::shared_ptr<basalt::RdSpline<3, N>> r3_traj;

public:
  TrajSE3(double time_interval, double start_time, size_t segment_id = 0)
    : basalt::Se3Spline<N, double>(time_interval, start_time) {
      std::cout << time_interval << " fuck you time_interval fuck you basalt" << std::endl;
    this->extendKnotsTo(start_time, Sophus::SO3<double>(Eigen::Quaterniond::Identity()),
                        Eigen::Vector3d(0, 0, 0));
  }
  std::shared_ptr<TrajSE3> getPtrInstance();

  decltype(auto) EvaluatePose(double time){
    return this->pose(time);
  }

  void setSO3Traj();

};