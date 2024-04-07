#pragma once

#include <Eigen/Dense>

namespace licalib {
struct Lidar {
private:
  double* li_time_offset;
  double q_LtoI[4];
  double p_LinI[3];

public:

  Lidar(){
    li_time_offset = new double(0);
    for(int i = 0; i < 3; i++) {
      q_LtoI[i]=0;
      p_LinI[i]=0;
    }
    q_LtoI[3]=0;
  };
  const size_t NumParameters() const {return 3;}
  double* time_offset() {return li_time_offset;}
  double* relative_position() {return q_LtoI;}
  double* relative_orientation() {return p_LinI;}


};
}
