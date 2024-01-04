#pragma once
#include "trajectory_estimator.h"
#include <spline/trajectory.h>

namespace estimator
{

class TrajectoryManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<TrajectoryManager> Ptr;

    TrajectoryManager(const YAML::Node &node, Trajectory::Ptr trajectory);

    void extendTrajectory(int64_t max_time);

private:
    Trajectory::Ptr trajectory_;

    int64_t max_bef_ns;
    int max_bef_idx;
    int max_aft_idx;
};

}
