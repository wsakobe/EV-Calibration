#pragma once

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include "factor/ceres_local_param.h"
#include <utils/parameter_struct.h>

namespace estimator{
class TrajectoryEstimator{
    static ceres::Problem::Options DefaultProblemOptions()
    {
      ceres::Problem::Options options;
      options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
      options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
      return options;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<TrajectoryEstimator> Ptr;

    TrajectoryEstimator(Trajectory::Ptr trajectory,
                        TrajectoryEstimatorOptions &option);

    ~TrajectoryEstimator()
    {
      // 手动删除new的变量
      delete analytic_local_parameterization_;
      // delete marginalization_info_;
    }

    /// 固定这帧之前的控制点
    void SetKeyScanConstant(double max_time);

    /// 直接指定固定控制点的索引
    void SetFixedIndex(int idx) { fixed_control_point_index_ = idx; }

    // [factor] 添加位置测量(起始位姿，固定轨迹的起始点)
    void AddStartTimePose(const PoseData &pose);

    // [factor] 轨迹固定因子
    void AddStaticSegment(const std::vector<size_t> &static_ctrl_segment);

    // [factor] 添加位置测量
    void AddPoseMeasurementAnalytic(const PoseData &pose_data,
                                    const Eigen::Matrix<double, 6, 1> &info_vec);

    // [factor] 图像重投影因子
    void AddImageFeatureAnalytic(const double ti, const Eigen::Vector3d &pi,
                                 const double tj, const Eigen::Vector3d &pj,
                                 double *inv_depth, bool fixed_depth = false,
                                 bool marg_this_fearure = false);

    // [factor] 图像重投影因子
    void AddImageFeatureDelayAnalytic(const int64_t ti, const int rowi, const Eigen::Vector3d &pi,
                                      const int64_t tj, const int rowj, const Eigen::Vector3d &pj,
                                      double *inv_depth, double *line_delay, bool fixed_depth = false,
                                      bool marg_this_fearure = false);

    void AddImageFeatureNew(const double ti, const int rowi, const Eigen::Vector3d &pi,
                            const double tj, const int rowj, const Eigen::Vector3d &pj,
                            double *inv_depth, bool fixed_depth = false,
                            bool marg_this_fearure = false);

    ceres::Solver::Summary Solve(int max_iterations = 50, bool progress = false,
                                 int num_threads = -1);

    std::shared_ptr<ceres::Problem> problem_;

private:
    void AddControlPoints(const SplineMeta<SplineOrder> &spline_meta,
                          std::vector<double *> &vec, bool addPosKnot = false);

    bool IsParamUpdated(const double *values) const;

private:
    Trajectory::Ptr trajectory_;

    // std::shared_ptr<ceres::Problem> problem_;
    ceres::LocalParameterization *analytic_local_parameterization_;

}

} //namespace estimator