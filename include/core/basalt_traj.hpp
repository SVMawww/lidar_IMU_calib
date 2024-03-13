// rewrite b-spline computing using basalt
// libaorun 23.12.22

#include <basalt/spline/so3_spline.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/spline/rd_spline.h>
#include <memory>

template <int N=4>
class TrajSE3 : public basalt::Se3Spline<N> {
using so3Traj = basalt::So3Spline<N>;

private: 
  std::shared_ptr<basalt::So3Spline<N>> so3_traj;
  std::shared_ptr<basalt::RdSpline<3, N>> r3_traj;

  TrajSE3(const std::shared_ptr<TrajSE3>);

public:
    
  std::shared_ptr<TrajSE3> getPtrInstance();

  decltype(auto) EvaluatePose(double time){
    return this->pose(time);
  }

  void setSO3Traj();

};