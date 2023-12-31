#pragma once

#include <utils/sophus_utils.hpp>
#include <utils/ceres_callbacks.h>
#include <utils/yaml_utils.h>

#include "initialization.h"

namespace estimator{

class EstimatorManager{
public:
    EstimatorManager(const YAML::Node& yaml_node, ros::NodeHandle& nh);
    ~EstimatorManager(){}
    
    void cornerArrayCallback(const corner_msgs::cornerArray& msg);
    void circleArrayCallback(const circle_msgs::circleArray& msg);

    void initialization();
    void optimization();

private:
    ros::NodeHandle nh_;
    ros::Subscriber corner_sub_;
    ros::Subscriber circle_sub_;

    Initializer est_initializer;
};

};
