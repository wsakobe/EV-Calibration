#pragma once

#include <utils/sophus_utils.hpp>
#include <utils/ceres_callbacks.h>
#include <utils/yaml_utils.h>

#include "initialization.h"
#include "trajectory_manager.h"

#include <mutex>

namespace estimator{

class EstimatorManager{
public:
    EstimatorManager(const YAML::Node& yaml_node, ros::NodeHandle& nh);
    ~EstimatorManager();
    
    void cornerArrayCallback(const corner_msgs::cornerArray& msg);
    void circleArrayCallback(const circle_msgs::circleArray& msg);

    void initialization();
    void optimization();

private:
    void performEstimator();
    void setInitialState();

    ros::NodeHandle nh_;
    ros::Subscriber corner_sub_;
    ros::Subscriber circle_sub_;

    Initializer est_initializer;
    TrajectoryManager::Ptr trajectory_manager_;
    Trajectory::Ptr trajectory_;

    std::vector<corner_msgs::cornerArray> corner_buffer_;
    std::vector<circle_msgs::circleArray> circle_buffer_;

    std::mutex init_lc;
};

};
