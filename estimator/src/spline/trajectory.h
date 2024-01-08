#pragma once

#include <glog/logging.h>
#include "../utils/parameter_struct.h"
#include <utils/mypcl_cloud_type.h>
#include "se3_spline.h"

namespace estimator
{
  class TrajectoryManager;

  class Trajectory : public Se3Spline<SplineOrder, double>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Trajectory> Ptr;

    Trajectory(double time_interval, double start_time = 0)
        : Se3Spline<SplineOrder, double>(time_interval * S_TO_NS, start_time * S_TO_NS),
          data_start_time_(-1)
    {
      this->extendKnotsTo(start_time, SO3d(Eigen::Quaterniond::Identity()), Eigen::Vector3d(0, 0, 0));
      std::cout << GREEN << "[init maxTime] " << this->maxTimeNs() * NS_TO_S << RESET << std::endl;
    }

    void SetDataStartTime(int64_t time) { data_start_time_ = time; }

    int64_t GetDataStartTime() { return data_start_time_; }

    bool fix_ld;
    double ld_lower;
    double ld_upper;
    SE3d T_ev2conv;

  private:
    int64_t data_start_time_;

    friend TrajectoryManager;

  };

} // namespace estimator
